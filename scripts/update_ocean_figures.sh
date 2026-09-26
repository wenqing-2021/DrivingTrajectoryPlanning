#!/usr/bin/env bash
# Refresh the OCEAN park-benchmark figures with the shared benchmark harness.
#
#   1. optionally rebuild (Release by default)
#   2. run the 14 benchmark cases through tests/benchmark/benchmark_planners.py,
#      each request limited to SOLVE_TIME_LIMIT seconds of solver.run
#   3. render each solved case to PNG and GIF, overwriting assets/ocean_benchmark
#
# Environment variables (all optional):
#   BUILD_TYPE=Release   SKIP_BUILD=1 (reuse the existing build)
#   SKIP_SOLVE=1 (reuse the existing solve_results/ocean_benchmark JSON)
#   SOLVE_TIME_LIMIT=15  WORKERS=16  REPEATS=1
#   PNG_DPI=120  GIF_DPI=300  RENDER_JOBS=4
#   RESULTS_DIRECTORY=...  FIGURE_DIRECTORY=...  STAGING_DIRECTORY=...
#   CASES="1 2 3 ..." (default: the 14 benchmark cases)
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
cd "${REPOSITORY_ROOT}"

BUILD_TYPE="${BUILD_TYPE:-Release}"
SKIP_BUILD="${SKIP_BUILD:-0}"
SKIP_SOLVE="${SKIP_SOLVE:-0}"
SOLVE_TIME_LIMIT="${SOLVE_TIME_LIMIT:-15}"
WORKERS="${WORKERS:-16}"
REPEATS="${REPEATS:-1}"
PNG_DPI="${PNG_DPI:-120}"
GIF_DPI="${GIF_DPI:-300}"
RENDER_JOBS="${RENDER_JOBS:-4}"
CASES=(${CASES:-1 2 3 4 5 6 8 11 12 13 14 16 17 18})
RESULTS_DIRECTORY="${RESULTS_DIRECTORY:-${REPOSITORY_ROOT}/solve_results/ocean_benchmark}"
FIGURE_DIRECTORY="${FIGURE_DIRECTORY:-${REPOSITORY_ROOT}/assets/ocean_benchmark}"
STAGING_DIRECTORY="${STAGING_DIRECTORY:-/tmp/ocean_figure_staging}"

for value in "${RENDER_JOBS}" "${WORKERS}" "${REPEATS}"; do
    if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
        echo "RENDER_JOBS, WORKERS and REPEATS must be positive integers" >&2
        exit 1
    fi
done

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${REPOSITORY_ROOT}/.cache/matplotlib}"
export PKG_CONFIG_PATH="${REPOSITORY_ROOT}/.local/ipopt/lib/pkgconfig:${REPOSITORY_ROOT}/.local/ipopt/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"

if [[ "${SKIP_BUILD}" != "1" ]]; then
    BUILD_TYPE="${BUILD_TYPE}" bash "${SCRIPT_DIRECTORY}/install/ensure_build_dependencies.sh"
    echo "==> Building (${BUILD_TYPE})"
    BUILD_TYPE="${BUILD_TYPE}" bash "${SCRIPT_DIRECTORY}/build.sh"
fi

JSON="${RESULTS_DIRECTORY}/ocean.json"
if [[ "${SKIP_SOLVE}" != "1" ]]; then
    echo "==> Solving ${#CASES[@]} case(s) with the ocean backend (${SOLVE_TIME_LIMIT}s limit each)"
    mkdir -p "${RESULTS_DIRECTORY}"
    uv run --frozen --no-sync python tests/benchmark/benchmark_planners.py \
        --backend ocean --cases "${CASES[@]}" --workers "${WORKERS}" --repeats "${REPEATS}" \
        --time-limit "${SOLVE_TIME_LIMIT}" --output "${JSON}"
fi

if [[ ! -f "${JSON}" ]]; then
    echo "Missing ${JSON}; run without SKIP_SOLVE=1 first" >&2
    exit 1
fi

# The renderer reads plan_problem.pb / plan_res.pb per case; the harness stores a
# SolverInput request and a PlanRes result, so split them into that layout.
echo "==> Preparing render inputs under ${RESULTS_DIRECTORY}/figure_input"
OCEAN_RESULTS_DIRECTORY="${RESULTS_DIRECTORY}" OCEAN_CASES="${CASES[*]}" \
    OCEAN_JSON_STEM="$(basename "${JSON}" .json)" \
    uv run --frozen --no-sync python - <<'PY'
import os
import sys
from pathlib import Path

ROOT = Path.cwd()
RESULTS = Path(os.environ["OCEAN_RESULTS_DIRECTORY"])
sys.path[:0] = [str(ROOT / path) for path in ("build/install/src", "build/install/src/protobuf")]
from problem_pb2 import PlanProblem, PlanRes, SolverInput  # noqa: E402

for case in os.environ["OCEAN_CASES"].split():
    prefix = os.environ["OCEAN_JSON_STEM"]
    request = RESULTS / f"{prefix}_case{case}_repeat0.request.pb"
    result = RESULTS / f"{prefix}_case{case}_repeat0.result.pb"
    destination = RESULTS / "figure_input" / f"Case{case}"
    if not request.is_file() or not result.is_file():
        print(f"    Case{case}: no result (failed or missing), skipped")
        continue
    parsed = PlanRes()
    parsed.ParseFromString(result.read_bytes())
    if not parsed.solve_success:
        print(f"    Case{case}: solver reported failure, skipped")
        continue
    destination.mkdir(parents=True, exist_ok=True)
    plan_problem = PlanProblem()
    plan_problem.ParseFromString(SolverInput.FromString(request.read_bytes()).plan_problem.SerializeToString())
    (destination / "plan_problem.pb").write_bytes(plan_problem.SerializeToString())
    (destination / "plan_res.pb").write_bytes(result.read_bytes())
PY

echo "==> Rendering figures to ${FIGURE_DIRECTORY} (${RENDER_JOBS} parallel cases)"
mkdir -p "${FIGURE_DIRECTORY}" "${STAGING_DIRECTORY}"
render_case() {
    local case_id="$1"
    local result_directory="${RESULTS_DIRECTORY}/figure_input/Case${case_id}"
    local staging_directory="${STAGING_DIRECTORY}/Case${case_id}"
    if [[ ! -f "${result_directory}/plan_res.pb" ]]; then
        echo "    Case${case_id}: no render input, skipped"
        return 0
    fi
    mkdir -p "${staging_directory}"
    echo "    Case${case_id}: rendering (log: ${staging_directory}/render.log)"
    bash scripts/visualize.sh "${result_directory}" --kind park --visualize png \
        --footprints --dpi "${PNG_DPI}" --output "${staging_directory}" >"${staging_directory}/render.log" 2>&1
    bash scripts/visualize.sh "${result_directory}" --kind park --visualize gif \
        --dpi "${GIF_DPI}" --output "${staging_directory}" >>"${staging_directory}/render.log" 2>&1
    cp "${staging_directory}/trajectory.png" "${FIGURE_DIRECTORY}/Case${case_id}.png"
    cp "${staging_directory}/trajectory.gif" "${FIGURE_DIRECTORY}/Case${case_id}.gif"
    echo "    Case${case_id}: PNG + GIF updated"
}
export -f render_case
export RESULTS_DIRECTORY FIGURE_DIRECTORY STAGING_DIRECTORY PNG_DPI GIF_DPI
printf '%s\0' "${CASES[@]}" | xargs -0 -n 1 -P "${RENDER_JOBS}" bash -c '
    set -eEuo pipefail
    trap '\''echo "    Case$1: rendering failed; check ${STAGING_DIRECTORY}/Case$1/render.log" >&2'\'' ERR
    render_case "$1"
' _
echo "==> Done. ${FIGURE_DIRECTORY} now holds $(find "${FIGURE_DIRECTORY}" -type f | wc -l) files"
