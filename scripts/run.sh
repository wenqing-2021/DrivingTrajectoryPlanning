#!/bin/bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
INSTALL_SOURCE="${REPOSITORY_ROOT}/build/install/src"
IPOPT_PREFIX="${IPOPT_PREFIX:-${REPOSITORY_ROOT}/.local/ipopt}"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export PYTHONPATH="${INSTALL_SOURCE}:${INSTALL_SOURCE}/protobuf:${REPOSITORY_ROOT}/build/pybind_modules:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${IPOPT_PREFIX}/lib:${IPOPT_PREFIX}/lib64:${LD_LIBRARY_PATH:-}"
RESULT_PATH="${RES_SAVE_PATH:-${REPOSITORY_ROOT}/solve_results/park/场景_$(date +%Y%m%d-%H%M%S)}"
cd "${INSTALL_SOURCE}"

# 1) Solve first and save pb files.
uv run python -m scenarios.park.main --res_save_path "${RESULT_PATH}" "$@"

# 2) Visualize saved pb files with vis_main.
uv run bokeh serve --show utils/visualization/vis_main.py \
    --args --res_save_path "${RESULT_PATH}"
