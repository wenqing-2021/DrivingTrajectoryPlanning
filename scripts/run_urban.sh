#!/bin/bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"

export PYTHONPATH="${REPOSITORY_ROOT}/src:${REPOSITORY_ROOT}/build/pybind_modules:${PYTHONPATH:-}"
cd "${REPOSITORY_ROOT}"
python3 -m scenarios.urban.main "$@"
