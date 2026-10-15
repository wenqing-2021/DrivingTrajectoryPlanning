"""Regression coverage for YAML loading with the protobuf upb runtime."""

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def solver_config_api(monkeypatch):
    """Load the same generated modules used by the installed Park entry point."""
    monkeypatch.syspath_prepend(str(ROOT / "build/install/src/protobuf"))
    monkeypatch.syspath_prepend(str(ROOT / "build/install/src"))
    from protobuf.params_pb2 import SolverParams
    from utils.plan_utils import convert_yaml_to_protobuf

    return SolverParams, convert_yaml_to_protobuf


def test_solver_config_loads_top_level_and_nested_fields(solver_config_api):
    solver_params_type, convert_yaml_to_protobuf = solver_config_api
    params = convert_yaml_to_protobuf(
        ROOT / "src/config/solver_params.yaml", solver_params_type()
    )

    assert params.map_resolution > 0
    assert params.hybrid_a_star_param.node_resolution_x > 0
    assert params.hybrid_a_star_param.node_resolution_y > 0
    assert params.hybrid_a_star_param.wheel_base > 0
    assert params.backend_solver == "ocean"


def test_explicit_zero_in_nested_config_preserves_presence(solver_config_api):
    solver_params_type, convert_yaml_to_protobuf = solver_config_api
    params = convert_yaml_to_protobuf(
        None,
        solver_params_type(),
        input_dict={"rda_params": {"safety_distance_step": 0}},
    )

    assert params.rda_params.HasField("safety_distance_step")
    assert params.rda_params.safety_distance_step == 0
