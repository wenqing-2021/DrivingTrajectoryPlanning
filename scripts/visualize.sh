#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export PYTHONPATH="${REPOSITORY_ROOT}/src:${PYTHONPATH:-}"
cd "${REPOSITORY_ROOT}"
uv run python -m utils.visualization.cli "$@"
