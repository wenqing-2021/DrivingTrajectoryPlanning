"""Tests for the Python CommonRoad urban integration."""

import numpy as np

from scenarios.urban.commonroad_scenario import create_demo_scenario
from scenarios.urban.config import default_config_path, load_config
from scenarios.urban.simulation import sample_local_reference


def test_demo_scenario_has_route_and_obstacle() -> None:
    urban_scenario = create_demo_scenario()

    assert urban_scenario.reference_path.shape[1] == 2
    assert len(urban_scenario.reference_path) > 10
    assert len(urban_scenario.obstacle_disks(0)) == 1
    np.testing.assert_allclose(urban_scenario.initial_state[:2], [2.0, 0.0])


def test_local_reference_matches_requested_horizon() -> None:
    reference_path = np.column_stack((np.linspace(0.0, 20.0, 21), np.zeros(21)))

    reference = sample_local_reference(
        reference_path,
        position=np.array([2.0, 0.0]),
        horizon=10,
        time_step=0.1,
        target_speed=5.0,
    )

    assert reference.shape == (11, 4)
    assert np.all(np.diff(reference[:, 0]) >= 0.0)
    np.testing.assert_allclose(reference[:-1, 3], 5.0)


def test_default_urban_config_loads() -> None:
    config = load_config(default_config_path())

    assert config.planner.horizon == 40
    assert config.planner.num_samples == 768
    assert config.simulation.max_steps == 120
