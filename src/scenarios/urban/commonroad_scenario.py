"""CommonRoad scenario creation, loading, and route extraction."""

import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
from commonroad.common.common_lanelet import LaneletType, LineMarking, RoadUser
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import (
    PlanningProblem,
    PlanningProblemSet,
)
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.state import CustomState, InitialState
from commonroad_route_planner.route_planner import RoutePlanner


@dataclass(frozen=True)
class ObstacleDisk:
    """Circular obstacle approximation passed to the C++ planner."""

    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class UrbanScenario:
    """CommonRoad objects and the extracted reference centerline."""

    scenario: Scenario
    planning_problem: PlanningProblem
    reference_path: np.ndarray

    @property
    def initial_state(self) -> np.ndarray:
        """Return the initial state as [x, y, yaw, velocity]."""
        state = self.planning_problem.initial_state
        return np.array(
            [
                state.position[0],
                state.position[1],
                state.orientation,
                state.velocity,
            ],
            dtype=float,
        )

    @property
    def goal_position(self) -> np.ndarray:
        """Return a representative goal center."""
        goal_state = self.planning_problem.goal.state_list[0]
        goal_shape = goal_state.position
        if hasattr(goal_shape, "center"):
            return np.asarray(goal_shape.center, dtype=float)
        if hasattr(goal_shape, "shapes") and goal_shape.shapes:
            return np.asarray(goal_shape.shapes[0].center, dtype=float)
        return self.reference_path[-1].copy()

    def obstacle_disks(self, time_step: int) -> List[ObstacleDisk]:
        """Return conservative circular occupancies at one simulation step."""
        disks = []
        for obstacle in self.scenario.obstacles:
            occupancy = obstacle.occupancy_at_time(time_step)
            if occupancy is None:
                continue
            shape = occupancy.shape
            center = np.asarray(shape.center, dtype=float)
            if hasattr(shape, "radius"):
                radius = float(shape.radius)
            elif hasattr(shape, "length") and hasattr(shape, "width"):
                radius = 0.5 * float(np.hypot(shape.length, shape.width))
            elif hasattr(shape, "vertices"):
                radius = float(
                    np.max(np.linalg.norm(np.asarray(shape.vertices) - center, axis=1))
                )
            else:
                continue
            disks.append(ObstacleDisk(float(center[0]), float(center[1]), radius))
        return disks


def _create_lanelet(
    center_y: float,
    lanelet_id: int,
    x_coordinates: np.ndarray,
    adjacent_left: Optional[int],
    adjacent_right: Optional[int],
) -> Lanelet:
    """Create one straight urban lanelet."""
    half_width = 1.75
    center_vertices = np.column_stack(
        (x_coordinates, np.full_like(x_coordinates, center_y))
    )
    left_vertices = center_vertices + np.array([0.0, half_width])
    right_vertices = center_vertices - np.array([0.0, half_width])
    return Lanelet(
        left_vertices=left_vertices,
        center_vertices=center_vertices,
        right_vertices=right_vertices,
        lanelet_id=lanelet_id,
        adjacent_left=adjacent_left,
        adjacent_left_same_direction=adjacent_left is not None,
        adjacent_right=adjacent_right,
        adjacent_right_same_direction=adjacent_right is not None,
        line_marking_left_vertices=(
            LineMarking.DASHED if adjacent_left else LineMarking.SOLID
        ),
        line_marking_right_vertices=(
            LineMarking.DASHED if adjacent_right else LineMarking.SOLID
        ),
        lanelet_type={LaneletType.URBAN},
        user_one_way={RoadUser.CAR},
    )


def _extract_reference_path(
    scenario: Scenario, planning_problem: PlanningProblem
) -> np.ndarray:
    """Extract and sanitize the first CommonRoad route centerline."""
    route_candidates = RoutePlanner(scenario, planning_problem).plan_routes()
    route = route_candidates.retrieve_best_route_by_orientation()
    if route is None:
        route = route_candidates.retrieve_first_route()
    if route is None:
        raise ValueError("CommonRoad route planner did not find a route")

    reference_path = np.asarray(route.reference_path, dtype=float)
    if reference_path.ndim != 2 or reference_path.shape[1] != 2:
        raise ValueError("CommonRoad route reference path must have shape [N, 2]")

    segment_lengths = np.linalg.norm(np.diff(reference_path, axis=0), axis=1)
    keep = np.concatenate(([True], segment_lengths > 1.0e-6))
    reference_path = reference_path[keep]
    if len(reference_path) < 2:
        raise ValueError("CommonRoad route reference path has fewer than two points")
    return reference_path


def create_demo_scenario() -> UrbanScenario:
    """Create a two-lane CommonRoad urban example with a parked obstacle."""
    scenario = Scenario(
        dt=0.1,
        scenario_id=ScenarioID(country_id="ZAM", map_name="MppiUrbanDemo", map_id=1),
        author="DrivingTrajectoryPlanning",
        source="Programmatic CommonRoad demo",
    )
    x_coordinates = np.linspace(0.0, 60.0, 31)
    scenario.add_objects(
        [
            _create_lanelet(
                0.0, 1, x_coordinates, adjacent_left=2, adjacent_right=None
            ),
            _create_lanelet(
                3.5, 2, x_coordinates, adjacent_left=None, adjacent_right=1
            ),
        ]
    )

    initial_state = InitialState(
        time_step=0,
        position=np.array([2.0, 0.0]),
        orientation=0.0,
        velocity=4.0,
        acceleration=0.0,
        yaw_rate=0.0,
        slip_angle=0.0,
    )
    goal_region = GoalRegion(
        [
            CustomState(
                time_step=Interval(0, 200),
                position=Rectangle(length=4.0, width=3.0, center=np.array([55.0, 0.0])),
                orientation=AngleInterval(-0.2, 0.2),
                velocity=Interval(0.0, 10.0),
            )
        ]
    )
    planning_problem = PlanningProblem(1, initial_state, goal_region)

    obstacle_state = InitialState(
        time_step=0,
        position=np.array([30.0, 0.0]),
        orientation=0.0,
        velocity=0.0,
        acceleration=0.0,
        yaw_rate=0.0,
        slip_angle=0.0,
    )
    scenario.add_objects(
        StaticObstacle(
            obstacle_id=10,
            obstacle_type=ObstacleType.CAR,
            obstacle_shape=Rectangle(length=4.5, width=2.0),
            initial_state=obstacle_state,
        )
    )

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        reference_path = _extract_reference_path(scenario, planning_problem)
    return UrbanScenario(scenario, planning_problem, reference_path)


def load_scenario(
    scenario_path: Path, planning_problem_id: Optional[int] = None
) -> UrbanScenario:
    """Load a CommonRoad XML scenario and choose one planning problem."""
    scenario, planning_problem_set = CommonRoadFileReader(str(scenario_path)).open()
    planning_problem = _select_planning_problem(
        planning_problem_set, planning_problem_id
    )
    reference_path = _extract_reference_path(scenario, planning_problem)
    return UrbanScenario(scenario, planning_problem, reference_path)


def _select_planning_problem(
    planning_problem_set: PlanningProblemSet,
    planning_problem_id: Optional[int],
) -> PlanningProblem:
    """Select a planning problem by ID or return the lowest-ID problem."""
    problems = planning_problem_set.planning_problem_dict
    if not problems:
        raise ValueError("CommonRoad file contains no planning problem")
    if planning_problem_id is not None:
        try:
            return problems[planning_problem_id]
        except KeyError as error:
            raise ValueError(
                f"Planning problem {planning_problem_id} is not present"
            ) from error
    return problems[min(problems)]
