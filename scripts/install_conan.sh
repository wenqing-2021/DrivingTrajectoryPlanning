#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
CONAN_OUTPUT_DIRECTORY="${REPOSITORY_ROOT}/build/conan"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
CONAN_CPPSTD="${CONAN_CPPSTD:-17}"
BUILD_TYPE_MARKER="${CONAN_OUTPUT_DIRECTORY}/.build-type"
REPOSITORY_ROOT_MARKER="${CONAN_OUTPUT_DIRECTORY}/.repository-root"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export CONAN_HOME="${CONAN_HOME:-${REPOSITORY_ROOT}/.cache/conan}"

if [[ ! -f "${CONAN_HOME}/profiles/default" ]]; then
    uv run --frozen conan profile detect --force
fi

uv run --frozen cmake -E remove_directory "${CONAN_OUTPUT_DIRECTORY}"

uv run --frozen conan install "${REPOSITORY_ROOT}" \
    --build=missing \
    --lockfile "${REPOSITORY_ROOT}/conan.lock" \
    --output-folder "${CONAN_OUTPUT_DIRECTORY}" \
    --settings "build_type=${BUILD_TYPE}" \
    --settings "compiler.cppstd=${CONAN_CPPSTD}"

printf '%s\n' "${BUILD_TYPE}" > "${BUILD_TYPE_MARKER}"
printf '%s\n' "${REPOSITORY_ROOT}" > "${REPOSITORY_ROOT_MARKER}"
