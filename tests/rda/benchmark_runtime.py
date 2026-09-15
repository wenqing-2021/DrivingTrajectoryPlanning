"""CPU RDA latency and footprint regression benchmark (run after scripts/build.sh).

uv run --frozen --no-sync python tests/rda/benchmark_runtime.py --output /tmp/rda.json
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / p)
    for p in ("build/install/src", "build/install/src/protobuf", "build/pybind_modules")
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cases",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4, 5, 6, 8, 11, 12, 13, 14, 16, 17, 18],
    )
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--workers", type=int, default=0, help="0 keeps the config value"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1 or args.workers < 0:
        parser.error("repeats must be positive and workers nonnegative")
    os.chdir(ROOT / "build/install/src")
    import numpy as np
    import solver_pybind
    from shapely.geometry import Polygon

    from protobuf.problem_pb2 import PlanRes, SolverInput
    from utils.plan_utils import build_problem, load_solver_params

    solver = solver_pybind.make_solver()
    params = load_solver_params(str(ROOT / "src/config/solver_params.yaml"))
    if not hasattr(params.rda_params, "workers"):
        raise RuntimeError("This build does not support RDA worker configuration")
    if args.workers > 0:
        params.rda_params.workers = args.workers
    effective_workers = params.rda_params.workers
    records = []
    for case in args.cases:
        problem = build_problem(
            str(ROOT / f"data/BenchmarkCases/Case{case}.csv"),
            str(ROOT / "src/config/vehicle_cfg.yaml"),
        )
        request = SolverInput(
            plan_problem=problem, solver_params=params
        ).SerializeToString()
        first_states = None
        for repeat in range(args.repeats):
            start = time.perf_counter()
            result = PlanRes.FromString(solver.run(request, False))
            elapsed = time.perf_counter() - start
            states = np.array([[s.x, s.y, s.theta, s.v] for s in result.init_traj])
            if first_states is None:
                first_states = states.copy()
            repeat_delta = (
                float(np.max(np.abs(states - first_states)))
                if states.shape == first_states.shape and states.size
                else float("inf")
            )
            vehicle = problem.vehicle_param
            obstacles = [
                Polygon([(v.x, v.y) for v in o.vertex_pts])
                for o in problem.obstacle_list
            ]
            local = [
                (-vehicle.rear_overhang, -vehicle.width / 2),
                (vehicle.length - vehicle.rear_overhang, -vehicle.width / 2),
                (vehicle.length - vehicle.rear_overhang, vehicle.width / 2),
                (-vehicle.rear_overhang, vehicle.width / 2),
            ]
            hits = 0
            for x, y, theta, _ in states:
                c, s = math.cos(theta), math.sin(theta)
                box = Polygon(
                    [(x + c * u - s * v, y + s * u + c * v) for u, v in local]
                )
                hits += any(box.intersects(o) for o in obstacles)
            goal = problem.goal_state
            error = (
                states[-1, :3] - [goal.x, goal.y, goal.theta]
                if len(states)
                else np.full(3, np.nan)
            )
            error[2] = math.atan2(math.sin(error[2]), math.cos(error[2]))
            dt = np.diff(result.init_timestamps)
            residual = None
            if len(dt) == len(states) - 1 and len(dt) and np.all(dt > 0):
                pred = states[:-1, :2] + dt[:, None] * states[
                    :-1, 3:4
                ] * np.column_stack((np.cos(states[:-1, 2]), np.sin(states[:-1, 2])))
                residual = float(np.max(np.abs(pred - states[1:, :2])))
            record = dict(
                case=case,
                repeat=repeat,
                workers=effective_workers,
                seconds=elapsed,
                success=result.solve_success,
                collisions=hits,
                endpoint_error=error.tolist(),
                repeat_max_state_delta=repeat_delta,
                xy_dynamics_residual=residual,
                states=states.tolist(),
                controls=[[c.accelerate, c.steer_angle] for c in result.init_controls],
                timestamps=list(result.init_timestamps),
            )
            records.append(record)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(records, indent=2))
            print(
                json.dumps(
                    {
                        k: v
                        for k, v in record.items()
                        if k not in ("states", "controls", "timestamps")
                    }
                ),
                file=sys.stderr,
                flush=True,
            )
    return (
        0
        if all(
            r["success"] and r["collisions"] == 0 and r["repeat_max_state_delta"] < 1e-8
            for r in records
        )
        else 1
    )


if __name__ == "__main__":
    sys.exit(main())
