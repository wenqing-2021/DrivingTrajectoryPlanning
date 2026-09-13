"""Scenario adapter tests, including nonzero CommonRoad time origins."""

from types import SimpleNamespace

import numpy as np
import pytest
from commonroad.geometry.shape import Circle, Rectangle, ShapeGroup
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory

from scenarios.urban.commonroad_scenario import create_demo_scenario
from scenarios.urban.simulation import SimulationResult
from utils.visualization.adapters.park import from_park
from utils.visualization.adapters.urban import from_urban, shape_polygons


def park_data():
    start = SimpleNamespace(x=0, y=0, theta=0, v=0)
    end = SimpleNamespace(x=2, y=1, theta=0.2, v=-1)
    problem = SimpleNamespace(
        init_state=start,
        goal_state=end,
        obstacle_list=[],
        vehicle_param=SimpleNamespace(
            length=4.5, width=2, rear_overhang=1, wheel_base=2.7
        ),
    )
    result = SimpleNamespace(
        solve_success=True,
        init_traj=[start, end],
        init_controls=[SimpleNamespace(accelerate=2, steer_angle=0.3)],
        pre_opt_traj=[],
        init_timestamps=[0, 0.17],
    )
    return problem, result


def test_park_actual_timing_and_legacy_fallback():
    problem, result = park_data()
    scene = from_park(problem, result, dt=0.1)
    np.testing.assert_allclose(scene.times, [0, 0.17])
    np.testing.assert_allclose(scene.controls, [[2, 0.3]])
    assert scene.vehicle.center_offset == 1.25
    assert scene.vehicle.wheel_base == 2.7
    np.testing.assert_allclose(
        np.mean(scene.vehicle.wheel_polygons([0, 0, 0])[:2], axis=(0, 1)), [0, 0]
    )
    result.init_timestamps = []
    assert from_park(problem, result).time_basis == "samples"
    with pytest.warns(UserWarning, match="assuming"):
        assumed = from_park(problem, result, dt=0.2)
    assert assumed.time_basis == "assumed"
    np.testing.assert_allclose(assumed.times, [0, 0.2])
    with pytest.raises(ValueError):
        from_park(problem, result, dt=-1)
    result.solve_success = False
    with pytest.raises(ValueError, match="search debug nodes"):
        from_park(problem, result)


def test_urban_dynamic_obstacle_time_alignment():
    urban = create_demo_scenario()
    urban.planning_problem.initial_state.time_step = 5
    initial = InitialState(
        time_step=5, position=np.array([10.0, 3.5]), orientation=0.0, velocity=1.0
    )
    following = InitialState(
        time_step=6, position=np.array([11.0, 3.5]), orientation=0.0, velocity=1.0
    )
    shape = Rectangle(4.0, 2.0)
    obstacle = DynamicObstacle(
        99,
        ObstacleType.CAR,
        shape,
        initial,
        TrajectoryPrediction(Trajectory(6, [following]), shape),
    )
    urban.scenario.add_objects(obstacle)
    result = SimulationResult(
        np.array([[2, 0, 0, 1], [3, 0, 0, 1], [4, 0, 0, 1]]),
        np.array([[0.2, 1], [0.3, 2]]),
        np.array([1, 2]),
        False,
    )
    scene = from_urban(urban, result)
    np.testing.assert_allclose(scene.times, [0.5, 0.6, 0.7])
    np.testing.assert_allclose(scene.controls, [[1, 0.2], [2, 0.3]])
    for time, step in ((scene.times[0], 5), (scene.times[1], 6)):
        np.testing.assert_allclose(
            scene.obstacles_at(time)[0], obstacle.occupancy_at_time(step).shape.vertices
        )
    assert scene.obstacles_at(scene.times[-1]) == []
    assert {p.kind for p in scene.layers} >= {
        "road",
        "boundary",
        "goal",
        "reference",
        "obstacle",
    }


def test_shape_group_and_circle():
    polygons = shape_polygons(
        ShapeGroup([Circle(1.0, np.zeros(2)), Rectangle(2.0, 1.0)])
    )
    assert len(polygons) == 2
    np.testing.assert_allclose(np.linalg.norm(polygons[0], axis=1), 1.0)
