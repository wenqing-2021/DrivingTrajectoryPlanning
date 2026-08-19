"""Python rendering for CommonRoad urban simulation results."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scenarios.urban.commonroad_scenario import UrbanScenario
from scenarios.urban.simulation import SimulationResult


def render_result(
    urban_scenario: UrbanScenario,
    result: SimulationResult,
    output_path: Path,
    *,
    show: bool = False,
) -> None:
    """Render the CommonRoad map, reference path, and driven trajectory."""
    from commonroad.visualization.mp_renderer import MPRenderer

    all_points = np.vstack((urban_scenario.reference_path, result.states[:, :2]))
    margin = 8.0
    plot_limits = [
        float(np.min(all_points[:, 0]) - margin),
        float(np.max(all_points[:, 0]) + margin),
        float(np.min(all_points[:, 1]) - margin),
        float(np.max(all_points[:, 1]) + margin),
    ]
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(12, 5))
    urban_scenario.scenario.draw(renderer)
    urban_scenario.planning_problem.draw(renderer)
    renderer.render()
    renderer.ax.plot(
        urban_scenario.reference_path[:, 0],
        urban_scenario.reference_path[:, 1],
        "--",
        color="tab:blue",
        linewidth=1.5,
        label="CommonRoad reference",
        zorder=20,
    )
    renderer.ax.plot(
        result.states[:, 0],
        result.states[:, 1],
        color="tab:red",
        linewidth=2.5,
        label="MPPI trajectory",
        zorder=21,
    )
    renderer.ax.scatter(
        result.states[0, 0],
        result.states[0, 1],
        color="tab:green",
        marker="o",
        label="start",
        zorder=10,
    )
    renderer.ax.legend(loc="upper right")
    renderer.ax.set_title(
        f"Urban MPPI simulation — goal reached: {result.reached_goal}"
    )
    renderer.ax.set_aspect("equal")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure = renderer.ax.figure
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(figure)


def _plot_curve(
    output_dir: Path,
    filename: str,
    title: str,
    xlabel: str,
    ylabel: str,
    x_values: np.ndarray,
    y_values: np.ndarray,
    color: str = "tab:blue",
) -> None:
    """Plot and save a single curve figure into the output directory."""
    output_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(x_values, y_values, color=color, linewidth=2)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / filename
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[renderer] saved {path}")


def render_control_curves(result: SimulationResult, output_dir: Path) -> None:
    """Render speed, acceleration, steering, and planning-cost curves."""
    state_time = np.arange(len(result.states))
    control_time = np.arange(len(result.controls))
    _plot_curve(
        output_dir,
        "speed.png",
        "Speed",
        "time step",
        "speed (m/s)",
        state_time,
        result.states[:, 3],
        color="tab:green",
    )
    _plot_curve(
        output_dir,
        "acceleration.png",
        "Acceleration",
        "time step",
        "acceleration (m/s\u00b2)",
        control_time,
        result.controls[:, 1],
        color="tab:orange",
    )
    _plot_curve(
        output_dir,
        "steering.png",
        "Steering angle",
        "time step",
        "steering angle (rad)",
        control_time,
        result.controls[:, 0],
        color="tab:purple",
    )
    if len(result.planning_costs) > 0:
        _plot_curve(
            output_dir,
            "planning_cost.png",
            "Planning cost",
            "time step",
            "planning cost",
            np.arange(len(result.planning_costs)),
            result.planning_costs,
            color="tab:red",
        )


def save_result_data(result: SimulationResult, output_dir: Path) -> None:
    """Save the raw trajectory, control, and cost arrays as a single .npz file."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "trajectory_controls.npz"
    np.savez(
        path,
        states=result.states,
        controls=result.controls,
        planning_costs=result.planning_costs,
        reached_goal=bool(result.reached_goal),
    )
    print(f"[renderer] saved {path}")
