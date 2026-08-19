# Scenario Architecture

The repository has two independent scenario front ends which share the CMake
build and common vehicle/planning utilities.

## Park

`scenarios.park.main` is the original parking pipeline. It reads benchmark CSV
polygons, builds the existing protobuf problem, calls `solver_pybind`, and
stores protobuf results for the Bokeh visualizer. `scripts/run.sh` invokes this
module directly; there is no root `src/main.py` compatibility entry point.

## Urban

`scenarios.urban` uses CommonRoad for the road network, planning problem,
obstacle occupancy, route generation, and Matplotlib renderer:

1. `commonroad_scenario.py` loads a CommonRoad XML file or creates the built-in
   two-lane demo. The CommonRoad route planner produces the reference path.
2. `config.py` loads simulation and MPPI parameters from
   `src/config/urban_mppi.yaml`.
3. `simulation.py` resamples a local reference and converts current CommonRoad
   obstacle occupancies to conservative circular envelopes.
4. `mppi_pybind` calls the C++ `planning::mppi::MppiPlanner`.
5. The first control is applied to a kinematic bicycle model and the process
   repeats as a receding-horizon simulation.
6. `renderer.py` draws the CommonRoad map, planning problem, reference path,
   and executed MPPI trajectory.

The C++ planner samples steering/acceleration perturbations, rolls out all
candidate controls, evaluates tracking, terminal, control, control-rate,
road-boundary, and collision costs, and applies the exponentially weighted
MPPI update. It shifts the optimized control sequence between simulation
steps for warm starting.

Trajectory smoothness and behavior are controlled by these planner parameters:

- `control_rate_weight` penalizes the squared change of steering/acceleration
  between consecutive controls (and against the previously applied control),
  which directly suppresses jagged control sequences.
- `noise_smoothing` (in `[0, 1)`) temporally correlates the sampled noise with
  an AR(1) process so each sampled rollout is smooth; higher values smooth the
  trajectory further, `0` restores classic per-step white-noise sampling.
- The MPPI importance weights use a self-calibrating temperature derived from
  the 10th-percentile cost excess. A fixed temperature collapses the weights
  onto a single random sample when the cumulative cost scale is large, which
  manifests as lateral weaving; the adaptive temperature keeps a stable set of
  samples contributing in every regime.
- `obstacle_softening` is the decay length of the exponential obstacle
  potential. Larger values make the repulsion act over a longer range with a
  gentler gradient, so the vehicle commits to a smooth detour early instead of
  reacting sharply at close range.
- `road_left_limit` / `road_right_limit` / `road_boundary_weight` define the
  drivable lateral corridor (meters, positive to the left of the reference
  heading) and penalize leaving it, keeping obstacle detours on the road
  surface.

## Commands

Build both pybind11 modules and all C++ tests:

```bash
bash scripts/build.sh
```

Run the existing park workflow:

```bash
bash scripts/run.sh
```

Run the built-in urban demo:

```bash
bash scripts/run_urban.sh
```

Run a CommonRoad XML scenario:

```bash
bash scripts/run_urban.sh --scenario path/to/scenario.xml \
  --config src/config/urban_mppi.yaml
```

Urban results are written to a timestamped scenario directory under
`solve_results/urban/场景_<solve time>/` (trajectory, control curves, and raw
data). Park results (protobuf files and rendered images) are written to
`solve_results/park/场景_<solve time>/`. MPPI sampling, horizon, vehicle limits,
cost weights, random seed, and simulation limits are configured in
`src/config/urban_mppi.yaml`.
