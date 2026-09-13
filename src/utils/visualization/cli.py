"""Export replay files or legacy Park/Urban results without running a solver."""

import argparse
from pathlib import Path

from utils.visualization.export import (
    DEFAULT_GIF_FPS,
    save_gif,
    save_png,
    validate_gif_fps,
)
from utils.visualization.model import Scene
from utils.visualization.renderer import RenderConfig


def _gif_fps_argument(value: str) -> float:
    try:
        return validate_gif_fps(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def add_render_arguments(
    parser: argparse.ArgumentParser, *, default: str = "none"
) -> None:
    """Share export arguments between scenario entry points and replay CLI."""
    parser.add_argument(
        "--visualize", choices=("none", "png", "gif", "both"), default=default
    )
    parser.add_argument(
        "--fps",
        type=_gif_fps_argument,
        default=DEFAULT_GIF_FPS,
        help="GIF simulation frequency, 10–100 Hz (default: 10; dt = 1/fps seconds).",
    )
    parser.add_argument("--playback-speed", type=float, default=1.0)
    parser.add_argument("--dpi", type=int, default=120)
    parser.add_argument(
        "--figsize", type=float, nargs=2, default=(12, 6), metavar=("WIDTH", "HEIGHT")
    )
    parser.add_argument(
        "--curves",
        action="store_true",
        help="Include synchronized curves in PNG (always enabled for GIF).",
    )
    parser.add_argument("--no-loop", action="store_true", help="Play the GIF once.")


def export_scene(
    scene: Scene,
    directory: Path,
    args,
    *,
    stem: str = "trajectory",
    time: float | None = None,
) -> None:
    """Write requested formats and always persist self-contained scene data."""
    config = RenderConfig(*args.figsize, dpi=args.dpi, curves=args.curves)
    directory = Path(directory)
    scene.save(directory / "replay.json")
    if args.visualize in ("png", "both"):
        save_png(scene, directory / f"{stem}.png", time=time, config=config)
    if args.visualize in ("gif", "both"):
        save_gif(
            scene,
            directory / f"{stem}.gif",
            fps=args.fps,
            speed=args.playback_speed,
            loop=not args.no_loop,
            config=config,
        )


def main(argv=None) -> int:
    """Replay a saved scene, with explicit adapters for older result directories."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="replay.json or a result directory")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--kind", choices=("park", "urban"), help="Legacy input type")
    parser.add_argument(
        "--dt", type=float, help="Assumed seconds per state for legacy Park data"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--scenario", type=Path, help="Original CommonRoad XML for legacy Urban data"
    )
    source.add_argument(
        "--demo", action="store_true", help="Explicitly use the built-in Urban scene"
    )
    parser.add_argument(
        "--time", type=float, help="PNG snapshot time; default is a full overview"
    )
    add_render_arguments(parser, default="both")
    args = parser.parse_args(argv)
    try:
        replay = args.input / "replay.json" if args.input.is_dir() else args.input
        if replay.is_file():
            scene = Scene.load(replay)
            if args.dt is not None and scene.time_basis == "samples":
                if args.dt <= 0:
                    raise ValueError("Assumed dt must be positive")
                scene.times = scene.times * args.dt
                scene.time_basis = "assumed"
                scene.__post_init__()
        elif args.kind == "park":
            from utils.visualization.adapters.park import from_park, load_messages

            scene = from_park(*load_messages(args.input), dt=args.dt)
        elif args.kind == "urban":
            if not args.scenario and not args.demo:
                raise ValueError("Legacy Urban data requires --scenario XML or --demo")
            import numpy as np

            from scenarios.urban.commonroad_scenario import (
                create_demo_scenario,
                load_scenario,
            )
            from scenarios.urban.simulation import SimulationResult
            from utils.visualization.adapters.urban import from_urban

            urban = (
                load_scenario(args.scenario)
                if args.scenario
                else create_demo_scenario()
            )
            with np.load(
                args.input / "trajectory_controls.npz", allow_pickle=False
            ) as data:
                result = SimulationResult(
                    data["states"],
                    data["controls"],
                    data["planning_costs"],
                    bool(data["reached_goal"]),
                )
            scene = from_urban(urban, result)
        else:
            raise ValueError(
                "Replay file missing; specify --kind for a legacy result directory"
            )
        output = args.output or (
            args.input if args.input.is_dir() else args.input.parent
        )
        export_scene(scene, output, args, time=args.time)
    except (ValueError, OSError, RuntimeError) as error:
        parser.error(str(error))
    print(f"Visualization saved to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
