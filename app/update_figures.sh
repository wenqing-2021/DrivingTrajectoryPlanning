#!/usr/bin/env bash
# Refresh the RDA park-benchmark figures.
#
#   1. build the project (Release by default)
#   2. re-run the benchmark cases, each limited to SOLVE_TIME_LIMIT seconds of
#      solver.run
#   3. render every successful case to PNG and GIF, overwriting the files in
#      assets/rda_benchmark
#
# Environment variables (all optional):
#   BUILD_TYPE=Release   SKIP_BUILD=1 (reuse the existing build)
#   SOLVE_TIME_LIMIT=10  PNG_DPI=120  GIF_DPI=60
#   RESULTS_DIRECTORY=...  FIGURE_DIRECTORY=...  STAGING_DIRECTORY=...
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
cd "${REPOSITORY_ROOT}"

BUILD_TYPE="${BUILD_TYPE:-Release}"
SKIP_BUILD="${SKIP_BUILD:-0}"
SOLVE_TIME_LIMIT="${SOLVE_TIME_LIMIT:-10}"
PNG_DPI="${PNG_DPI:-120}"
GIF_DPI="${GIF_DPI:-60}"
CASES=(1 2 3 4 5 6 8 11 12 13 14 16 17 18)
RESULTS_DIRECTORY="${RESULTS_DIRECTORY:-${REPOSITORY_ROOT}/solve_results/rda_benchmark_rerun}"
FIGURE_DIRECTORY="${FIGURE_DIRECTORY:-${REPOSITORY_ROOT}/assets/rda_benchmark}"
STAGING_DIRECTORY="${STAGING_DIRECTORY:-/tmp/rda_figure_staging}"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${REPOSITORY_ROOT}/.cache/matplotlib}"
export PKG_CONFIG_PATH="${REPOSITORY_ROOT}/.local/ipopt/lib/pkgconfig:${REPOSITORY_ROOT}/.local/ipopt/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"
export RDA_CASES="${CASES[*]}"
export RDA_TIME_LIMIT="${SOLVE_TIME_LIMIT}"

if [[ "${SKIP_BUILD}" != "1" ]]; then
    echo "==> Building (${BUILD_TYPE})"
    BUILD_TYPE="${BUILD_TYPE}" bash scripts/build.sh
fi

echo "==> Running ${#CASES[@]} benchmark cases (${SOLVE_TIME_LIMIT}s solve limit each)"
mkdir -p "${RESULTS_DIRECTORY}"
cat > "${RESULTS_DIRECTORY}/run_cases.py" <<'PY'
"""Sequential RDA benchmark run; the POSIX timer covers only solver.run."""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
CASES = [int(case) for case in os.environ["RDA_CASES"].split()]
LIMIT = float(os.environ["RDA_TIME_LIMIT"])

sys.path[:0] = [str(ROOT / path) for path in
                ("build/install/src", "build/install/src/protobuf", "build/pybind_modules")]


def worker(case):
    import solver_pybind
    from protobuf.problem_pb2 import PlanRes, SolverInput
    from protobuf.cost_map_pb2 import CostMap
    from utils.plan_utils import build_problem, load_solver_params

    destination = OUT / ("Case" + str(case))
    destination.mkdir(exist_ok=True)
    problem = build_problem(str(ROOT / ("data/BenchmarkCases/Case" + str(case) + ".csv")),
                            str(ROOT / "src/config/vehicle_cfg.yaml"))
    params = load_solver_params(str(ROOT / "src/config/solver_params.yaml"))
    request = SolverInput()
    request.plan_problem.CopyFrom(problem)
    request.solver_params.CopyFrom(params)
    payload = request.SerializeToString()
    (destination / "plan_problem.pb").write_bytes(problem.SerializeToString())

    solver = solver_pybind.make_solver()
    signal.signal(signal.SIGALRM, signal.SIG_DFL)
    start = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, LIMIT)
    result_payload = solver.run(payload, False)
    elapsed = time.perf_counter() - start
    signal.setitimer(signal.ITIMER_REAL, 0)

    result = PlanRes()
    result.ParseFromString(result_payload)
    cost_map = CostMap()
    cost_map.ParseFromString(solver.get_cost_map())
    result.cost_map.CopyFrom(cost_map)
    (destination / "plan_res.pb").write_bytes(result.SerializeToString())
    (destination / "result.json").write_text(json.dumps(
        {"case": "Case" + str(case), "solve_success": result.solve_success,
         "trajectory_points": len(result.init_traj), "solve_seconds": elapsed}, indent=2))


def collision_summary():
    """Discrete vehicle-footprint check so the run reports which cases still hit."""
    try:
        from shapely.geometry import Polygon
        from protobuf.problem_pb2 import PlanProblem, PlanRes
    except ImportError as error:
        print("    (collision summary skipped: " + str(error) + ")")
        return
    print("==> Collision summary (discrete states)")
    for case in CASES:
        destination = OUT / ("Case" + str(case))
        if not (destination / "plan_res.pb").exists():
            print("    Case" + str(case) + ": no result")
            continue
        problem = PlanProblem()
        problem.ParseFromString((destination / "plan_problem.pb").read_bytes())
        result = PlanRes()
        result.ParseFromString((destination / "plan_res.pb").read_bytes())
        if not result.solve_success:
            print("    Case" + str(case) + ": solve failed")
            continue
        obstacles = [Polygon([(v.x, v.y) for v in o.vertex_pts]) for o in problem.obstacle_list]
        vehicle = problem.vehicle_param
        half_width = vehicle.width / 2.0
        local = [(-vehicle.rear_overhang, -half_width), (vehicle.length - vehicle.rear_overhang, -half_width),
                 (vehicle.length - vehicle.rear_overhang, half_width), (-vehicle.rear_overhang, half_width)]
        hits = 0
        for state in result.init_traj:
            cosine, sine = math.cos(state.theta), math.sin(state.theta)
            footprint = Polygon([(state.x + cosine * px - sine * py, state.y + sine * px + cosine * py)
                                 for px, py in local])
            if any(footprint.intersects(obstacle) for obstacle in obstacles):
                hits += 1
        status = "clean" if hits == 0 else str(hits) + "/" + str(len(result.init_traj)) + " states collide"
        print("    Case" + str(case) + ": " + status)


if len(sys.argv) == 2:
    worker(int(sys.argv[1]))
else:
    environment = os.environ.copy()
    environment["PYTHONPATH"] = ":".join(str(ROOT / path) for path in
                                        ("build/install/src", "build/install/src/protobuf",
                                         "build/pybind_modules"))
    environment["LD_LIBRARY_PATH"] = ":".join([str(ROOT / ".local/ipopt/lib"),
                                               str(ROOT / ".local/ipopt/lib64"),
                                               environment.get("LD_LIBRARY_PATH", "")])
    for case in CASES:
        completed = subprocess.run([sys.executable, str(Path(__file__).resolve()), str(case)],
                                   env=environment, cwd=ROOT / "build/install/src",
                                   timeout=LIMIT * 6)
        print("Case" + str(case) + ": process rc=" + str(completed.returncode), flush=True)
    collision_summary()
PY
uv run --frozen --no-sync python "${RESULTS_DIRECTORY}/run_cases.py"

echo "==> Rendering figures to ${FIGURE_DIRECTORY}"
mkdir -p "${FIGURE_DIRECTORY}" "${STAGING_DIRECTORY}"
for case_id in "${CASES[@]}"; do
    result_directory="${RESULTS_DIRECTORY}/Case${case_id}"
    if [[ ! -f "${result_directory}/plan_res.pb" ]]; then
        echo "    Case${case_id}: no result (failed or timed out), skipped"
        continue
    fi
    bash scripts/visualize.sh "${result_directory}" --kind park --visualize png \
        --dpi "${PNG_DPI}" --output "${STAGING_DIRECTORY}/Case${case_id}" >/dev/null
    cp "${STAGING_DIRECTORY}/Case${case_id}/trajectory.png" "${FIGURE_DIRECTORY}/Case${case_id}.png"
    bash scripts/visualize.sh "${result_directory}" --kind park --visualize gif \
        --dpi "${GIF_DPI}" --output "${STAGING_DIRECTORY}/Case${case_id}" >/dev/null
    cp "${STAGING_DIRECTORY}/Case${case_id}/trajectory.gif" "${FIGURE_DIRECTORY}/Case${case_id}.gif"
    echo "    Case${case_id}: PNG + GIF updated"
done
echo "==> Done. ${FIGURE_DIRECTORY} now holds $(find "${FIGURE_DIRECTORY}" -type f | wc -l) files"
