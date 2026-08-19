#!/bin/bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
BUILD_DIRECTORY="${REPOSITORY_ROOT}/build"

mkdir -p "${BUILD_DIRECTORY}"
cmake \
    -S "${REPOSITORY_ROOT}" \
    -B "${BUILD_DIRECTORY}" \
    -DCMAKE_INSTALL_PREFIX="${BUILD_DIRECTORY}/install"
cmake --build "${BUILD_DIRECTORY}" --parallel 4
cmake --install "${BUILD_DIRECTORY}"
