import numpy as np
import pytest
from benchmark_planners import smoothness_metrics


def test_physical_derivatives_on_nonuniform_times():
    times = np.array([0, 0.08, 0.25, 0.4])
    controls = np.column_stack((2 * times[:-1], 3 * times[:-1] ** 2))
    metrics = smoothness_metrics(controls, times)
    assert metrics["jerk_rms"] == pytest.approx(2)
    assert metrics["jerk_energy"] == pytest.approx(1)
    assert metrics["steering_acceleration_rms"] == pytest.approx(6)
    assert metrics["steering_acceleration_energy"] == pytest.approx(4.5)
    slower = smoothness_metrics(controls, times * 2)
    assert slower["jerk_rms"] == pytest.approx(metrics["jerk_rms"] / 2)
    assert slower["steering_rate_rms"] == pytest.approx(
        metrics["steering_rate_rms"] / 2
    )
    assert slower["steering_acceleration_rms"] == pytest.approx(1.5)
    assert slower["steering_acceleration_energy"] == pytest.approx(4.5 / 8)


def test_constant_commands_have_no_derivative_cost():
    metrics = smoothness_metrics([[0.5, 0.1]] * 3, [0, 0.1, 0.3, 0.5])
    assert metrics["jerk_energy"] == 0
    assert metrics["steering_rate_energy"] == 0
    assert metrics["steering_acceleration_energy"] == 0
