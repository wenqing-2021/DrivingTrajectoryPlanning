"""Compare parking planners with independent geometry and dynamics checks.

Run after a Release build. Cases execute sequentially in one persistent Solver.
Example: uv run --frozen --no-sync python tests/benchmark/benchmark_planners.py \
    --backend ocean --output solve_results/ocean_benchmark/ocean.json
"""

import argparse
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CASES = [1, 2, 3, 4, 5, 6, 8, 11, 12, 13, 14, 16, 17, 18]


def validate(problem, states, controls, timestamps, dt_min, dt_max, bound_margin=1e-3):
    """Independent geometry, dynamics and limit checks for one result.

    Collision and map checks are exact: they are safety properties. Inequality
    bounds (endpoint box, control and time limits) are compared with an explicit
    numerical margin, and the achieved excess is reported next to the verdict so
    a marginal overshoot is visible instead of being hidden by the tolerance.
    """
    import numpy as np
    from shapely.geometry import Polygon, box

    s, u, t = map(np.asarray, (states, controls, timestamps))
    empty = dict(
        quality_pass=False,
        failures=["invalid_trajectory_shape_or_values"],
        endpoint_errors=[],
        endpoint_excess=0.0,
        steering_excess=0.0,
        acceleration_excess=0.0,
        speed_excess=0.0,
        time_excess=0.0,
        dynamics_max=[0.0] * 4,
        dense_collisions=0,
        samples=0,
        boundary_violations=0,
        min_clearance=None,
        control_min=[],
        control_max=[],
    )
    if (
        s.ndim != 2
        or s.shape[1:] != (4,)
        or len(s) < 2
        or u.shape != (len(s) - 1, 2)
        or t.shape != (len(s),)
        or not all(np.isfinite(a).all() for a in (s, u, t))
    ):
        return empty
    dt = np.diff(t)
    if abs(t[0]) > 1e-8 or (dt <= 0).any():
        return {**empty, "failures": ["invalid_timestamps"]}
    v = problem.vehicle_param
    errors = []
    endpoint_errors = []
    endpoint_excess = 0.0
    for k, target in [(0, problem.init_state), (-1, problem.goal_state)]:
        e = s[k] - [target.x, target.y, target.theta, target.v]
        e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        endpoint_errors.append(e.tolist())
        endpoint_excess = max(
            endpoint_excess,
            abs(e[0]) - 0.05,
            abs(e[1]) - 0.05,
            abs(e[2]) - 0.08,
            abs(e[3]) - 1e-3,
        )
    if endpoint_excess > bound_margin:
        errors.append("endpoint")
    steering_excess = float(max(0.0, np.abs(u[:, 1]).max() - v.max_steer_angle))
    acceleration_excess = float(max(0.0, np.abs(u[:, 0]).max() - v.max_acc))
    speed_excess = float(max(0.0, np.abs(s[:, 3]).max() - v.max_velocity))
    time_excess = float(max(0.0, dt_min - dt.min(), dt.max() - dt_max))
    for name, excess in (
        ("steering_bound", steering_excess),
        ("acceleration_bound", acceleration_excess),
        ("speed_bound", speed_excess),
        ("time_bound", time_excess),
    ):
        if excess > bound_margin:
            errors.append(name)
    predicted = s[:-1] + dt[:, None] * np.column_stack(
        (
            s[:-1, 3] * np.cos(s[:-1, 2]),
            s[:-1, 3] * np.sin(s[:-1, 2]),
            s[:-1, 3] * np.tan(u[:, 1]) / v.wheel_base,
            u[:, 0],
        )
    )
    residual = predicted - s[1:]
    residual[:, 2] = np.arctan2(np.sin(residual[:, 2]), np.cos(residual[:, 2]))
    dynamics = np.max(abs(residual), axis=0)
    if (dynamics > 1e-3).any():
        errors.append("dynamics")
    local = np.array(
        [
            [-v.rear_overhang, -v.width / 2],
            [v.length - v.rear_overhang, -v.width / 2],
            [v.length - v.rear_overhang, v.width / 2],
            [-v.rear_overhang, v.width / 2],
        ]
    )
    obstacles = [
        Polygon([(p.x, p.y) for p in o.vertex_pts]) for o in problem.obstacle_list
    ]
    b = problem.map_bound
    boundary = box(b.min_x, b.min_y, b.max_x, b.max_y)
    radius = np.max(np.linalg.norm(local, axis=1))
    collisions = boundary_violations = samples = 0
    min_clearance = math.inf
    for k, state in enumerate(s):
        difference = s[k + 1] - state if k + 1 < len(s) else np.zeros(4)
        difference[2] = math.atan2(math.sin(difference[2]), math.cos(difference[2]))
        parts = max(
            1,
            math.ceil(
                (np.linalg.norm(difference[:2]) + radius * abs(difference[2])) / 0.05
            ),
        )
        for j in range(parts):
            pose = state + j / parts * difference
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            footprint = Polygon(local @ np.array([[c, sn], [-sn, c]]) + pose[:2])
            samples += 1
            collisions += any(footprint.intersects(o) for o in obstacles)
            boundary_violations += not boundary.covers(footprint)
            for obstacle in obstacles:
                min_clearance = min(min_clearance, footprint.distance(obstacle))
    if collisions:
        errors.append("collision")
    if boundary_violations:
        errors.append("map_boundary")
    return dict(
        quality_pass=not errors,
        failures=sorted(set(errors)),
        endpoint_errors=endpoint_errors,
        endpoint_excess=endpoint_excess,
        steering_excess=steering_excess,
        acceleration_excess=acceleration_excess,
        speed_excess=speed_excess,
        time_excess=time_excess,
        dynamics_max=dynamics.tolist(),
        dense_collisions=collisions,
        samples=samples,
        boundary_violations=boundary_violations,
        min_clearance=min_clearance if math.isfinite(min_clearance) else None,
        control_min=u.min(axis=0).tolist(),
        control_max=u.max(axis=0).tolist(),
        bound_margin=bound_margin,
    )


def git_read(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["rda", "ocean"], required=True)
    parser.add_argument("--cases", nargs="+", type=int, default=CASES)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "src/config/solver_params.yaml"
    )
    parser.add_argument("--time-limit", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (
        args.workers < 0
        or args.repeats < 1
        or not math.isfinite(args.time_limit)
        or args.time_limit <= 0
    ):
        parser.error("Invalid workers, repeats or time limit")
    args.output = args.output.resolve()
    args.config = args.config.resolve()
    sys.path[:0] = [
        str(ROOT / p)
        for p in (
            "build/install/src",
            "build/install/src/protobuf",
            "build/pybind_modules",
        )
    ]
    import numpy as np
    import solver_pybind
    from google.protobuf.json_format import MessageToDict

    from protobuf.problem_pb2 import PlanRes, SolverInput
    from utils.plan_utils import build_problem, load_solver_params

    os.chdir(ROOT / "build/install/src")
    params = load_solver_params(str(args.config))
    params.backend_solver = args.backend
    backend_params = getattr(params, args.backend + "_params")
    backend_params.workers = args.workers
    if args.backend == "ocean":
        backend_params.time_limit = args.time_limit
    metadata = dict(
        backend=args.backend,
        config=MessageToDict(params, preserving_proto_field_name=True),
        vehicle_yaml=(ROOT / "src/config/vehicle_cfg.yaml").read_text(),
        commit=git_read("rev-parse", "HEAD"),
        dirty=bool(git_read("status", "--porcelain")),
        tracked_diff_hash=hashlib.sha256(git_read("diff", "HEAD").encode()).hexdigest(),
        platform=platform.platform(),
        cpu=platform.processor(),
        cpu_count=os.cpu_count(),
        repeats=args.repeats,
        time_limit=args.time_limit,
        cases=args.cases,
        timing=(
            "solver.run including frontend and setup; validation excluded; "
            "no process hard kill"
        ),
    )
    source_paths = git_read(
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "src",
        "protobuf",
        "tests",
        "CMakeLists.txt",
    ).splitlines()
    source_hash = hashlib.sha256()
    for name in sorted(source_paths):
        path = ROOT / name
        if path.is_file():
            source_hash.update(name.encode())
            source_hash.update(path.read_bytes())
    metadata["source_tree_hash"] = source_hash.hexdigest()
    metadata["build_type"] = (ROOT / "build/conan/.build-type").read_text().strip()
    metadata["cpu_model"] = next(
        (
            line.split(":", 1)[1].strip()
            for line in Path("/proc/cpuinfo").read_text().splitlines()
            if line.startswith("model name")
        ),
        "unknown",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    records = []
    solver = solver_pybind.make_solver()
    for case in args.cases:
        case_path = ROOT / f"data/BenchmarkCases/Case{case}.csv"
        problem = build_problem(
            str(case_path), str(ROOT / "src/config/vehicle_cfg.yaml")
        )
        request = SolverInput(
            plan_problem=problem, solver_params=params
        ).SerializeToString()
        first = None
        for repeat in range(args.repeats):
            start = time.perf_counter()
            result = PlanRes.FromString(solver.run(request, False))
            elapsed = time.perf_counter() - start
            states = (
                [[s.x, s.y, s.theta, s.v] for s in result.init_traj]
                if result.solve_success
                else []
            )
            controls = (
                [[c.accelerate, c.steer_angle] for c in result.init_controls]
                if result.solve_success
                else []
            )
            timestamps = list(result.init_timestamps)
            if repeat == 0:
                first = (states, controls, timestamps)
            consistent = all(
                np.shape(a) == np.shape(b) and np.allclose(a, b, atol=1e-7, rtol=1e-6)
                for a, b in zip((states, controls, timestamps), first, strict=True)
            )
            check_start = time.perf_counter()
            quality = validate(
                problem,
                states,
                controls,
                timestamps,
                backend_params.dt_min,
                backend_params.dt_max,
            )
            diagnostic = MessageToDict(
                result.diagnostics, preserving_proto_field_name=True
            )
            candidate = {
                key: diagnostic.pop(key)
                for key in list(diagnostic)
                if key.startswith("candidate_")
            }
            record = dict(
                case=case,
                repeat=repeat,
                workers=args.workers,
                solver_success=result.solve_success,
                seconds=elapsed,
                within_budget=elapsed <= args.time_limit,
                repeat_consistent=bool(consistent),
                validation_seconds=time.perf_counter() - check_start,
                diagnostics=diagnostic,
                states=states,
                controls=controls,
                timestamps=timestamps,
                case_hash=hashlib.sha256(case_path.read_bytes()).hexdigest(),
                **quality,
            )
            if candidate:
                record["failed_candidate"] = candidate
                d = result.diagnostics
                record["candidate_quality"] = validate(
                    problem,
                    [[s.x, s.y, s.theta, s.v] for s in d.candidate_states],
                    [[c.accelerate, c.steer_angle] for c in d.candidate_controls],
                    list(d.candidate_timestamps),
                    backend_params.dt_min,
                    backend_params.dt_max,
                )
            record["pass"] = bool(
                result.solve_success
                and quality["quality_pass"]
                and consistent
                and record["within_budget"]
            )
            records.append(record)
            # Prefix with the output stem so separate runs in one directory do not
            # overwrite each other's per-case payloads.
            stem = args.output.parent / f"{args.output.stem}_case{case}_repeat{repeat}"
            stem.with_suffix(".request.pb").write_bytes(request)
            stem.with_suffix(".result.pb").write_bytes(result.SerializeToString())
            args.output.write_text(
                json.dumps(
                    dict(metadata=metadata, records=records), indent=2, allow_nan=False
                )
            )
            print(
                json.dumps(
                    {
                        k: v
                        for k, v in record.items()
                        if k
                        not in ("states", "controls", "timestamps", "failed_candidate")
                    }
                ),
                flush=True,
            )
    lines = [
        f"# {args.backend} benchmark",
        "",
        f"workers={args.workers} time-limit={args.time_limit}s "
        f"repeats={args.repeats} commit={metadata['commit']} "
        f"dirty={metadata['dirty']} "
        f"bound-margin={records[0].get('bound_margin')}",
        "",
        "| Case | Pass | Solver success | Median s | Range s | Dynamics max |"
        " Min clearance | Failures |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for case in args.cases:
        rows = [r for r in records if r["case"] == case]
        times = [r["seconds"] for r in rows]
        dynamics = max(max(r["dynamics_max"]) for r in rows)
        clearance = min(
            (r["min_clearance"] for r in rows if r["min_clearance"] is not None),
            default=float("nan"),
        )
        failures = ", ".join(sorted({f for r in rows for f in r["failures"]})) or "-"
        lines.append(
            f"| {case} | {sum(r['pass'] for r in rows)}/{len(rows)} | "
            f"{sum(r['solver_success'] for r in rows)}/{len(rows)} | "
            f"{statistics.median(times):.4f} | "
            f"{min(times):.4f}-{max(times):.4f} | {dynamics:.2e} | "
            f"{clearance:.4f} | {failures} |"
        )
    lines += [
        "",
        "Total median seconds: "
        f"{statistics.median([r['seconds'] for r in records]):.4f}",
        "Total seconds: " f"{sum(r['seconds'] for r in records):.4f}",
    ]
    args.output.with_suffix(".md").write_text("\n".join(lines) + "\n")
    return 0 if all(r["pass"] for r in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
