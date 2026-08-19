"""Closed-loop CommonRoad simulation driven by the C++ MPPI planner."""

import importlib
from dataclasses import dataclass, fields
from typing import Any, List

import numpy as np

from scenarios.urban.commonroad_scenario import UrbanScenario
from scenarios.urban.config import UrbanConfig


@dataclass(frozen=True)
class SimulationResult:
    """State and control history of one urban simulation."""

    states: np.ndarray
    controls: np.ndarray
    planning_costs: np.ndarray
    reached_goal: bool


def _load_mppi_module() -> Any:
    """Load the CMake-built pybind11 module with an actionable error."""
    try:
        return importlib.import_module("mppi_pybind")
    except ImportError as error:
        raise RuntimeError(
            "mppi_pybind is unavailable. Run 'bash scripts/build.sh' and add "
            "'build/pybind_modules' to PYTHONPATH."
        ) from error


def _arc_lengths(path: np.ndarray) -> np.ndarray:
    return np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    )


def sample_local_reference(
    reference_path: np.ndarray,
    position: np.ndarray,
    horizon: int,
    time_step: float,
    target_speed: float,
) -> np.ndarray:
    """Sample a horizon-length [x, y, yaw, speed] route reference."""
    path = np.asarray(reference_path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        raise ValueError("reference_path must have shape [N, 2] with N >= 2")

    arc_lengths = _arc_lengths(path)
    nearest_index = int(np.argmin(np.linalg.norm(path - position[:2], axis=1)))
    start_distance = arc_lengths[nearest_index]
    sample_distances = np.minimum(
        start_distance + np.arange(horizon + 1, dtype=float) * target_speed * time_step,
        arc_lengths[-1],
    )
    x_coordinates = np.interp(sample_distances, arc_lengths, path[:, 0])
    y_coordinates = np.interp(sample_distances, arc_lengths, path[:, 1])
    edge_order = 2 if len(sample_distances) > 2 else 1
    yaw = np.arctan2(
        np.gradient(y_coordinates, edge_order=edge_order),
        np.gradient(x_coordinates, edge_order=edge_order),
    )
    velocity = np.full(horizon + 1, target_speed)
    velocity[sample_distances >= arc_lengths[-1] - 1.0e-6] = 0.0
    return np.column_stack((x_coordinates, y_coordinates, yaw, velocity))


def _binding_states(module: Any, values: np.ndarray) -> List[Any]:
    states = []
    for x_coordinate, y_coordinate, yaw, velocity in values:
        state = module.VehicleState()
        state.x = float(x_coordinate)
        state.y = float(y_coordinate)
        state.yaw = float(yaw)
        state.velocity = float(velocity)
        states.append(state)
    return states


def _binding_obstacles(
    module: Any, urban_scenario: UrbanScenario, time_step: int
) -> List[Any]:
    obstacles = []
    for disk in urban_scenario.obstacle_disks(time_step):
        obstacle = module.CircularObstacle()
        obstacle.x = disk.x
        obstacle.y = disk.y
        obstacle.radius = disk.radius
        obstacles.append(obstacle)
    return obstacles


def run_simulation(
    urban_scenario: UrbanScenario,
    config: UrbanConfig,
) -> SimulationResult:
    """Run receding-horizon MPPI against CommonRoad obstacle occupancies."""
    module = _load_mppi_module()
    native_config = module.MppiConfig()
    for config_field in fields(config.planner):
        setattr(
            native_config,
            config_field.name,
            getattr(config.planner, config_field.name),
        )
    native_config.time_step = float(urban_scenario.scenario.dt)
    planner = module.MppiPlanner(native_config)

    initial = urban_scenario.initial_state
    current_state = _binding_states(module, initial.reshape(1, 4))[0]
    states = [initial]
    controls = []
    planning_costs = []
    reached_goal = False

    for time_step in range(config.simulation.max_steps):
        local_reference = sample_local_reference(
            urban_scenario.reference_path,
            np.array([current_state.x, current_state.y]),
            config.planner.horizon,
            native_config.time_step,
            config.simulation.target_speed,
        )
        result = planner.plan(
            current_state,
            _binding_states(module, local_reference),
            _binding_obstacles(module, urban_scenario, time_step),
        )
        if len(result.states) < 2 or not result.controls:
            raise RuntimeError("MPPI returned an empty rollout")

        applied_control = result.controls[0]
        current_state = result.states[1]
        states.append(
            np.array(
                [
                    current_state.x,
                    current_state.y,
                    current_state.yaw,
                    current_state.velocity,
                ]
            )
        )
        controls.append(
            np.array([applied_control.steering, applied_control.acceleration])
        )
        planning_costs.append(result.cost)

        distance_to_goal = np.linalg.norm(states[-1][:2] - urban_scenario.goal_position)
        if distance_to_goal <= config.simulation.goal_tolerance:
            reached_goal = True
            break

    return SimulationResult(
        states=np.asarray(states),
        controls=np.asarray(controls),
        planning_costs=np.asarray(planning_costs),
        reached_goal=reached_goal,
    )
