# Shared Urban / Park visualization

Both scenario front ends use the same Matplotlib scene renderer to produce PNG
images and GIF animations. Existing Park styles and rear-axle geometry are reused;
`MatplotlibVis.draw_main()` and Urban's `render_result()` delegate to this renderer.
Park's Bokeh viewer and ESDF/control-curve figures remain available.

## Run and export

Build once after this change to regenerate protobufs, compile timestamp export,
and install the Python sources used by `scripts/run_park.sh`:

```bash
bash scripts/build.sh
bash scripts/run_urban.sh --visualize both --curves
bash scripts/run_park.sh --visualize both --curves
```

`--visualize` accepts `none`, `png`, `gif`, or `both`. Urban defaults to PNG.
Park keeps its original Bokeh workflow when this option is omitted; specifying it
makes `scripts/run_park.sh` a batch run without starting a server.
Urban also saves its separate control/cost PNGs when PNG export is selected.

Outputs go to the existing result directory:

| Output | Urban | Park |
| --- | --- | --- |
| Static overview | `trajectory.png` | `main.png` |
| Animation | `trajectory.gif` | `main.gif` |
| Self-contained scene | `replay.json` | `replay.json` |

Every successful Park solve with a result directory saves replay data, including
when scene image export is disabled. Urban also saves replay data on completion,
including runs which have not reached the goal.

## Replay without solving

```bash
bash scripts/visualize.sh solve_results/urban/<run> --visualize both
bash scripts/visualize.sh solve_results/park/<run>/replay.json --visualize gif
bash scripts/visualize.sh solve_results/park/<run> --visualize png --time 2.5 \
  --output solve_results/snapshot
```

The replay CLI defaults to both formats and uses `trajectory.png` /
`trajectory.gif` in its output directory. Specify `--output` to keep an existing
export. `--time` selects a PNG snapshot instead of the full trajectory overview;
it does not truncate GIF playback. Without `--time`, dynamic obstacles and the
vehicle are shown at the final recorded time, with the complete ego trajectory.

Shared rendering options:

- `--fps 10`: simulation sampling frequency, default/minimum 10 Hz, maximum 100 Hz.
  The interpolation interval is `1 / fps` seconds: 10 Hz gives 0.1 s and 20 Hz
  gives 0.05 s, independently of the source trajectory's time intervals.
- `--playback-speed 2`: double-speed playback; simulation timing is unchanged.
- `--dpi 150 --figsize 12 6`: image resolution and width/height in inches.
- GIFs always include synchronized speed, acceleration and steering panels where available.
- `--curves`: also include these panels in PNG output.
- `--no-loop`: play the GIF once; otherwise it repeats indefinitely.

Vehicles display four tires. Rear tires follow the body heading; front tires follow
the recorded steering command, held over each control interval and at the final
pose. Missing controls display straight tires. Park uses its vehicle wheelbase and
rear overhang; new Urban runs record the configured wheelbase. Older replays without
axle dimensions use a display wheelbase of 60% of body length with symmetric
overhangs. Tire size and track width are display proportions; both front tires use
the bicycle model's steering angle.

GIF frames interpolate x, y, speed and unwrapped heading on a fixed `1 / fps`
simulation-time grid starting at the first timestamp. If the last interval is
shorter than `1 / fps`, the exact terminal state is appended; earlier intervals
remain unchanged. Playback speed changes frame delays, not this sampling grid.
The product of frequency and playback speed must not exceed 100 frames/s.

GIF intervals are quantized to 10 ms. The movement duration matches recorded
time divided by playback speed within this quantization; one extra display frame
holds the terminal pose. Frames include both endpoints. Occupancies use the
previous recorded scene step, while ego positions and unwrapped heading are
interpolated. GIF export retains indexed-color frames in memory; for long runs,
reduce FPS (down to 10 Hz) or DPI. No display server, GPU, browser, or FFmpeg is required.

Urban display geometry can be set at simulation export using `--vehicle-length`,
`--vehicle-width`, and `--vehicle-center-offset`. Defaults are 4.5 m, 2.0 m, and
zero offset from the planner state. These are display parameters; MPPI continues
to use its configured circular collision envelope.

## Legacy results

Old Park protobufs have no timestamps. They can produce a static overview directly:

```bash
bash scripts/visualize.sh solve_results/park/<old-run> --kind park --visualize png
```

For GIF export, provide an explicit assumed interval:

```bash
bash scripts/visualize.sh solve_results/park/<old-run> --kind park \
  --dt 0.1 --visualize both
```

This is labeled as **assumed timing** and is not a recovery of RDA's actual
variable intervals. Without timing, static plots use sample indices.
Failed Park results contain search debug nodes; they are rejected as replay input.

Old Urban NPZ files do not include scene geometry. Supply the original CommonRoad
XML, or explicitly select the built-in demo if that was the original scene:

```bash
bash scripts/visualize.sh solve_results/urban/<old-run> --kind urban \
  --scenario path/to/original.xml
bash scripts/visualize.sh solve_results/urban/<old-demo-run> --kind urban --demo
```

Once `replay.json` exists, it is preferred over legacy inputs. A replay contains
map/goal polygons, named paths, vehicle geometry, trajectory, controls and sampled
dynamic occupancies, so the original XML and native solver are no longer required.
It captures visualization geometry, not the complete CommonRoad scenario schema.

## Architecture and timing

- `model.py`: validated scene data, vehicle geometry, interpolation, JSON version 1.
- `adapters/park.py`: protobuf conversion and legacy timing handling.
- `adapters/urban.py`: CommonRoad roads, shapes, goals and obstacle snapshots.
- `style.py`: styles extracted from the original Park Matplotlib implementation.
- `renderer.py`: shared static layers and frame updates, independent of planners.
- `export.py`: PNG/GIF encoding and playback timing.
- `cli.py`: replay entry point and shared export options.

Coordinates are meters, heading/steering radians, timestamps seconds, and controls
are `[acceleration, steering]`. Optional speed and controls can be omitted. States
have N samples; controls can have N-1 or N samples, or be empty. An N-1 control
sequence has no new command at the terminal state.

The Park protobuf field `init_timestamps` records optimized trajectory timing and
is additive and backward-compatible. Optimized RDA timestamps accumulate its actual
per-segment intervals. OBCA currently uses its
0.1 s solver interval; PWJ uses its configured interval. New Park results reconstruct
acceleration as `(v[i+1] - v[i]) / dt[i]` and steering as
`atan(wheelbase * wrapped_heading_change / (v[i] * dt[i]))`, using signed velocity
and the actual optimized timestamps. Initial steering is zero; below 1e-4 m/s,
steering holds its previous value because it cannot be inferred at rest. Optimized
positions, heading and velocity remain unchanged. This reconstruction matches the
RDA/OBCA forward-Euler bicycle model; it does not repair an infeasible state
trajectory or enforce steering-rate constraints. Visualization reads the exported
controls; old result files must be regenerated to obtain corrected controls.

Urban timestamps include the planning problem's initial time step. Both simulation
obstacle lookup and replay lookup use that same origin. Missing dynamic obstacle
occupancy is rendered as absent, including after its prediction ends.

## Validation

```bash
uv run pytest -q tests/visualization tests/commonroad
uv run ctest --test-dir build/cmake --output-on-failure
```

Tests cover geometry offsets, reversing, heading wrap, nonuniform timing, missing
controls, single/invalid trajectories, obstacle disappearance, CommonRoad time
origins, portable replay round trips, GIF frame timing, and synchronized curves.
