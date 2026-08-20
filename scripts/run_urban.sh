#!/bin/bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
IPOPT_PREFIX="${IPOPT_PREFIX:-${REPOSITORY_ROOT}/.local/ipopt}"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export PYTHONPATH="${REPOSITORY_ROOT}/src:${REPOSITORY_ROOT}/build/pybind_modules:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${IPOPT_PREFIX}/lib:${IPOPT_PREFIX}/lib64:${LD_LIBRARY_PATH:-}"
cd "${REPOSITORY_ROOT}"
uv run python -m scenarios.urban.main "$@"
