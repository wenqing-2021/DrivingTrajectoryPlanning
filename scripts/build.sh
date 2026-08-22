#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
BUILD_ROOT="${REPOSITORY_ROOT}/build"
BUILD_DIRECTORY="${BUILD_ROOT}/cmake"
CONAN_OUTPUT_DIRECTORY="${BUILD_ROOT}/conan"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
BUILD_JOBS="${BUILD_JOBS:-4}"
IPOPT_PREFIX="${IPOPT_PREFIX:-${REPOSITORY_ROOT}/.local/ipopt}"
OSQP_PREFIX="${OSQP_PREFIX:-${REPOSITORY_ROOT}/.local/osqp}"
OSQP_INSTALL_MARKER="${OSQP_PREFIX}/.installed-osqp-1.0.0-osqp-eigen-0.11.2-${BUILD_TYPE}-pic"
BUILD_REPOSITORY_MARKER="${BUILD_DIRECTORY}/.repository-root"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"
export PKG_CONFIG_PATH="${IPOPT_PREFIX}/lib/pkgconfig:${IPOPT_PREFIX}/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"

missing_dependencies=()
if ! command -v g++ >/dev/null 2>&1; then
    missing_dependencies+=("C++ compiler")
fi
if ! command -v pkg-config >/dev/null 2>&1; then
    missing_dependencies+=("pkg-config")
elif [[ "$(pkg-config --variable=prefix ipopt 2>/dev/null || true)" != "${IPOPT_PREFIX}" ]]; then
    missing_dependencies+=("Ipopt installed under ${IPOPT_PREFIX}")
fi
osqp_install_prefix=""
if [[ -f "${OSQP_INSTALL_MARKER}" ]]; then
    IFS= read -r osqp_install_prefix < "${OSQP_INSTALL_MARKER}" || true
fi
if [[ "${osqp_install_prefix}" != "${OSQP_PREFIX}" ]] \
    || [[ ! -f "${OSQP_PREFIX}/lib/cmake/osqp/osqp-config.cmake" ]] \
    || [[ ! -f "${OSQP_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake" ]]; then
    missing_dependencies+=("OSQP 1.0.0 and OsqpEigen 0.11.2 installed under ${OSQP_PREFIX}")
fi
if [[ ! -f "${CONAN_OUTPUT_DIRECTORY}/conan_toolchain.cmake" \
    || ! -f "${CONAN_OUTPUT_DIRECTORY}/.build-type" \
    || ! -f "${CONAN_OUTPUT_DIRECTORY}/.repository-root" ]]; then
    missing_dependencies+=("Conan dependency environment")
else
    conan_build_type=""
    IFS= read -r conan_build_type < "${CONAN_OUTPUT_DIRECTORY}/.build-type" || true
    if [[ "${conan_build_type}" != "${BUILD_TYPE}" ]]; then
        missing_dependencies+=("Conan dependency environment for ${BUILD_TYPE}")
    fi
    conan_repository_root=""
    IFS= read -r conan_repository_root < "${CONAN_OUTPUT_DIRECTORY}/.repository-root" || true
    if [[ "${conan_repository_root}" != "${REPOSITORY_ROOT}" ]]; then
        missing_dependencies+=("Conan environment for this repository path")
    fi
fi
if [[ ! -f /usr/include/cppad/cppad.hpp ]]; then
    missing_dependencies+=("CppAD headers")
fi
if (( ${#missing_dependencies[@]} > 0 )); then
    echo "Missing dependencies: ${missing_dependencies[*]}" >&2
    echo "Run 'bash scripts/setup.sh' to install them." >&2
    exit 1
fi

configured_repository_root=""
if [[ -f "${BUILD_REPOSITORY_MARKER}" ]]; then
    IFS= read -r configured_repository_root < "${BUILD_REPOSITORY_MARKER}" || true
fi
if [[ "${configured_repository_root}" != "${REPOSITORY_ROOT}" ]]; then
    # Compiler dependency files also store absolute paths, so a relocated
    # checkout needs a fresh generated build tree.
    uv run --frozen --no-sync cmake -E remove_directory "${BUILD_DIRECTORY}"
fi

uv run --frozen --no-sync cmake --fresh \
    -S "${REPOSITORY_ROOT}" \
    -B "${BUILD_DIRECTORY}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${BUILD_ROOT}/install" \
    -DCMAKE_PREFIX_PATH="${OSQP_PREFIX}" \
    -DCMAKE_BUILD_RPATH="${IPOPT_PREFIX}/lib;${IPOPT_PREFIX}/lib64" \
    -DCMAKE_INSTALL_RPATH="${IPOPT_PREFIX}/lib;${IPOPT_PREFIX}/lib64" \
    -DCMAKE_TOOLCHAIN_FILE="${CONAN_OUTPUT_DIRECTORY}/conan_toolchain.cmake"
uv run --frozen --no-sync cmake --build "${BUILD_DIRECTORY}" --parallel "${BUILD_JOBS}"
uv run --frozen --no-sync cmake --install "${BUILD_DIRECTORY}"
printf '%s\n' "${REPOSITORY_ROOT}" > "${BUILD_REPOSITORY_MARKER}"
