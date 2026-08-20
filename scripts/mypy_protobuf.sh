#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
CONAN_ENV="${REPOSITORY_ROOT}/build/conan/conanbuild.sh"
export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"

if [[ ! -f "${CONAN_ENV}" ]]; then
    echo "Run 'bash scripts/setup.sh' before generating protobuf stubs." >&2
    exit 1
fi

mkdir -p "${REPOSITORY_ROOT}/src/protobuf"

cd "${REPOSITORY_ROOT}"
source "${CONAN_ENV}"
uv run protoc \
    --proto_path=protobuf \
    --mypy_out=src/protobuf \
    protobuf/*.proto
