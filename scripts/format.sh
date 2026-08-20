#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
cd "${REPOSITORY_ROOT}"

uv run black src tests data/BenchmarkCases/RunMe.py

find src protobuf \
    -type f \( -name '*.cc' -o -name '*.cpp' -o -name '*.h' \) \
    -not -path '*/third-party/*' \
    -exec uv run clang-format -i {} +
