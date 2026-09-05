"""End-to-end test for the C++ MPPI urban simulation."""

import numpy as np
import pytest

from scenarios.urban.commonroad_scenario import create_demo_scenario
from scenarios.urban.config import default_config_path, load_config
from scenarios.urban.simulation import run_simulation

pytest.importorskip("mppi_pybind")


def test_mppi_reaches_goal_without_intersecting_obstacle_envelope() -> None:
    urban_scenario = create_demo_scenario()

    result = run_simulation(urban_scenario, load_config(default_config_path()))

    obstacle = urban_scenario.obstacle_disks(0)[0]
    center_distances = np.hypot(
        result.states[:, 0] - obstacle.x,
        result.states[:, 1] - obstacle.y,
    )
    assert result.reached_goal
    assert np.min(center_distances) > obstacle.radius + 1.2
