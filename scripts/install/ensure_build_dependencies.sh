#!/usr/bin/env bash
# Make Conan dependencies and the OSQP stack match BUILD_TYPE.
#
# build/conan and .local/osqp hold one build type at a time, so a Debug setup
# followed by a Release build (or the reverse) fails build.sh's dependency
# check. setup.sh and update_figures.sh call this script first so the requested
# build type is always self-consistent, and already matching dependencies are
# left untouched.
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
CONAN_OUTPUT_DIRECTORY="${REPOSITORY_ROOT}/build/conan"
OSQP_PREFIX="${OSQP_PREFIX:-${REPOSITORY_ROOT}/.local/osqp}"
OSQP_INSTALL_MARKER="${OSQP_PREFIX}/.installed-osqp-1.0.0-osqp-eigen-0.11.2-${BUILD_TYPE}-pic"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"

conan_build_type=""
conan_repository_root=""
if [[ -f "${CONAN_OUTPUT_DIRECTORY}/.build-type" ]]; then
    IFS= read -r conan_build_type < "${CONAN_OUTPUT_DIRECTORY}/.build-type" || true
fi
if [[ -f "${CONAN_OUTPUT_DIRECTORY}/.repository-root" ]]; then
    IFS= read -r conan_repository_root < "${CONAN_OUTPUT_DIRECTORY}/.repository-root" || true
fi
if [[ "${conan_build_type}" != "${BUILD_TYPE}" \
    || "${conan_repository_root}" != "${REPOSITORY_ROOT}" \
    || ! -f "${CONAN_OUTPUT_DIRECTORY}/conan_toolchain.cmake" ]]; then
    echo "==> Conan dependencies target ${conan_build_type:-nothing}; installing for ${BUILD_TYPE}"
    BUILD_TYPE="${BUILD_TYPE}" bash "${SCRIPT_DIRECTORY}/install_conan.sh"
fi

osqp_install_prefix=""
if [[ -f "${OSQP_INSTALL_MARKER}" ]]; then
    IFS= read -r osqp_install_prefix < "${OSQP_INSTALL_MARKER}" || true
fi
if [[ "${osqp_install_prefix}" != "${OSQP_PREFIX}" \
    || ! -f "${OSQP_PREFIX}/lib/cmake/osqp/osqp-config.cmake" \
    || ! -f "${OSQP_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake" ]]; then
    echo "==> OSQP and OsqpEigen are not installed for ${BUILD_TYPE}; building them"
    BUILD_TYPE="${BUILD_TYPE}" bash "${SCRIPT_DIRECTORY}/install_osqp.sh"
fi

echo "==> Build dependencies for ${BUILD_TYPE} are ready"
