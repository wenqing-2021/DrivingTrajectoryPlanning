"""Command-line entry point for CommonRoad urban MPPI simulation."""

import argparse
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

from scenarios.urban.commonroad_scenario import create_demo_scenario, load_scenario
from scenarios.urban.config import default_config_path, load_config
from scenarios.urban.renderer import (
    render_control_curves,
    save_result_data,
)
from scenarios.urban.simulation import run_simulation
from utils.visualization.adapters.urban import from_urban
from utils.visualization.cli import add_render_arguments, export_scene
from utils.visualization.model import Vehicle

_DEFAULT_RESULT_ROOT = Path("solve_results/urban")


def _default_output_dir() -> Path:
    """Return a timestamped directory under solve_results/urban."""
    solve_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    return _DEFAULT_RESULT_ROOT / f"场景_{solve_time}"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse urban simulation arguments."""
    parser = argparse.ArgumentParser("CommonRoad urban MPPI simulation")
    parser.add_argument(
        "--scenario",
        type=Path,
        help="CommonRoad XML file. Omit to run the built-in two-lane demo.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="Urban simulation and MPPI YAML configuration.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output directory for the trajectory, control curves, and data "
            "(default: solve_results/urban/场景_<solve time>)."
        ),
    )
    add_render_arguments(parser, default="png")
    parser.add_argument(
        "--vehicle-length",
        type=float,
        default=4.5,
        help="Displayed vehicle length in meters.",
    )
    parser.add_argument("--vehicle-width", type=float, default=2.0)
    parser.add_argument(
        "--vehicle-center-offset",
        type=float,
        default=0.0,
        help="Display body center offset from planner position (m).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run an urban scenario and render its trajectory."""
    args = parse_args(argv)
    output_dir = args.output if args.output is not None else _default_output_dir()
    urban_scenario = (
        load_scenario(args.scenario) if args.scenario else create_demo_scenario()
    )
    config = load_config(args.config)
    result = run_simulation(urban_scenario, config)

    scene = from_urban(
        urban_scenario,
        result,
        vehicle=Vehicle(
            args.vehicle_length,
            args.vehicle_width,
            args.vehicle_center_offset,
            wheel_base=config.planner.wheelbase,
        ),
    )
    save_result_data(result, output_dir)
    export_scene(scene, output_dir, args)
    if args.visualize in ("png", "both"):
        render_control_curves(result, output_dir)

    final_state = result.states[-1]
    print(
        "Urban simulation complete: "
        f"steps={len(result.controls)}, reached_goal={result.reached_goal}, "
        f"final_position=({final_state[0]:.2f}, {final_state[1]:.2f})"
    )
    print(f"Results saved to: {output_dir}")
    return 0 if result.reached_goal else 2


if __name__ == "__main__":
    raise SystemExit(main())
