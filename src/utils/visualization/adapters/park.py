"""Adapt Park protobuf results, including legacy results without timestamps."""

import importlib
import sys
import warnings
from pathlib import Path

import numpy as np

from utils.visualization.model import PathLayer, Scene, Vehicle


def load_messages(directory: Path):
    """Load generated protobufs in both source and CMake install layouts."""
    for parent in Path(__file__).resolve().parents:
        for candidate in (parent / "protobuf", parent / "build/install/src/protobuf"):
            if (candidate / "problem_pb2.py").is_file():
                sys.path.insert(0, str(candidate))
                module = importlib.import_module("problem_pb2")
                problem, result = module.PlanProblem(), module.PlanRes()
                problem.ParseFromString((directory / "plan_problem.pb").read_bytes())
                result.ParseFromString((directory / "plan_res.pb").read_bytes())
                return problem, result
    raise RuntimeError("Generated Park protobufs missing; run bash scripts/build.sh")


def from_park(problem, result, *, dt: float | None = None) -> Scene:
    """Convert solved Park results; search debug nodes are not a drivable path."""
    if not result.solve_success:
        raise ValueError("Park solve failed; search debug nodes cannot be replayed")
    states = np.array([[s.x, s.y, s.theta, s.v] for s in result.init_traj])
    times = np.asarray(getattr(result, "init_timestamps", []), dtype=float)
    basis = "recorded"
    if not len(times):
        if dt is not None and (not np.isfinite(dt) or dt <= 0):
            raise ValueError("Assumed dt must be finite and positive")
        basis = "assumed" if dt is not None else "samples"
        times = np.arange(len(states)) * (dt if dt is not None else 1.0)
        if dt is not None:
            warnings.warn(
                f"Park timestamps missing; assuming dt={dt:g} s", stacklevel=2
            )
    vp = problem.vehicle_param
    layers = [
        PathLayer([[p.x, p.y] for p in obstacle.vertex_pts], kind="obstacle")
        for obstacle in problem.obstacle_list
    ]
    if len(result.pre_opt_traj) >= 2:
        layers.append(
            PathLayer(
                [[s.x, s.y] for s in result.pre_opt_traj], "Pre-optimization", "initial"
            )
        )
    return Scene(
        states=states,
        times=times,
        vehicle=Vehicle(
            vp.length,
            vp.width,
            vp.length / 2 - vp.rear_overhang,
            wheel_base=getattr(vp, "wheel_base", 0) or None,
            rear_overhang=vp.rear_overhang,
        ),
        controls=[[c.accelerate, c.steer_angle] for c in result.init_controls],
        layers=layers,
        start=[problem.init_state.x, problem.init_state.y, problem.init_state.theta],
        goal=[problem.goal_state.x, problem.goal_state.y, problem.goal_state.theta],
        title="Park trajectory",
        time_basis=basis,
        metadata={
            "scenario": "park",
            "pose_reference": "rear_axle",
            "controls_source": "Park result controls",
        },
    )
