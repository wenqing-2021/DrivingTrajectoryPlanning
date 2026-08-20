#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
SOURCE_ROOT="${REPOSITORY_ROOT}/.cache/osqp/sources"
BUILD_ROOT="${REPOSITORY_ROOT}/.cache/osqp/build"
OSQP_PREFIX="${OSQP_PREFIX:-${REPOSITORY_ROOT}/.local/osqp}"
CONAN_OUTPUT_DIRECTORY="${REPOSITORY_ROOT}/build/conan"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
BUILD_JOBS="${BUILD_JOBS:-4}"
INSTALL_MARKER="${OSQP_PREFIX}/.installed-osqp-1.0.0-osqp-eigen-0.11.2-${BUILD_TYPE}-pic"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"

for command_name in gcc g++ tar uv wget sha256sum; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Missing OSQP build tool: ${command_name}" >&2
        echo "Run 'bash scripts/setup.sh' to install host prerequisites." >&2
        exit 1
    fi
done

if [[ -f "${INSTALL_MARKER}" ]]; then
    installed_prefix=""
    IFS= read -r installed_prefix < "${INSTALL_MARKER}" || true
    if [[ "${installed_prefix}" == "${OSQP_PREFIX}" ]]; then
        echo "OSQP 1.0.0 and OsqpEigen 0.11.2 are already installed."
        exit 0
    fi
fi

if [[ ! -f "${CONAN_OUTPUT_DIRECTORY}/conan_toolchain.cmake" \
    || ! -f "${CONAN_OUTPUT_DIRECTORY}/.build-type" \
    || ! -f "${CONAN_OUTPUT_DIRECTORY}/.repository-root" ]]; then
    echo "Missing Conan dependency environment." >&2
    echo "Run 'bash scripts/install_conan.sh' first." >&2
    exit 1
fi
conan_build_type=""
IFS= read -r conan_build_type < "${CONAN_OUTPUT_DIRECTORY}/.build-type" || true
if [[ "${conan_build_type}" != "${BUILD_TYPE}" ]]; then
    echo "Conan dependencies were not installed for ${BUILD_TYPE}." >&2
    echo "Run 'BUILD_TYPE=${BUILD_TYPE} bash scripts/install_conan.sh' first." >&2
    exit 1
fi
conan_repository_root=""
IFS= read -r conan_repository_root < "${CONAN_OUTPUT_DIRECTORY}/.repository-root" || true
if [[ "${conan_repository_root}" != "${REPOSITORY_ROOT}" ]]; then
    echo "Conan dependencies belong to a different repository path." >&2
    echo "Run 'bash scripts/install_conan.sh' first." >&2
    exit 1
fi

mkdir -p "${SOURCE_ROOT}" "${BUILD_ROOT}"

download_and_extract() {
    local name="$1"
    local url="$2"
    local checksum="$3"
    local extracted_directory="$4"
    local archive="${SOURCE_ROOT}/${name}.tar.gz"

    if [[ ! -f "${archive}" ]]; then
        wget --quiet --show-progress --output-document="${archive}" "${url}"
    fi
    echo "${checksum}  ${archive}" | sha256sum --check --status
    if [[ ! -d "${SOURCE_ROOT}/${extracted_directory}" ]]; then
        tar -xf "${archive}" -C "${SOURCE_ROOT}"
    fi
}

download_and_extract \
    qdldl-0.1.8 \
    https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz \
    ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa \
    qdldl-0.1.8
download_and_extract \
    osqp-1.0.0 \
    https://github.com/osqp/osqp/archive/refs/tags/v1.0.0.tar.gz \
    dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48 \
    osqp-1.0.0
download_and_extract \
    osqp-eigen-0.11.2 \
    https://github.com/gbionics/osqp-eigen/archive/refs/tags/v0.11.2.tar.gz \
    93f165468be4e717748e7c3d09f41b4f0fd640fcc350ee4b7f0779dad9968e03 \
    osqp-eigen-0.11.2

uv run --frozen cmake -E remove_directory "${BUILD_ROOT}/osqp"
uv run --frozen cmake -E remove_directory "${BUILD_ROOT}/osqp-eigen"
uv run --frozen cmake -E remove_directory "${OSQP_PREFIX}"
mkdir -p "${OSQP_PREFIX}"

uv run --frozen cmake \
    -S "${SOURCE_ROOT}/osqp-1.0.0" \
    -B "${BUILD_ROOT}/osqp" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${OSQP_PREFIX}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DFETCHCONTENT_SOURCE_DIR_QDLDL="${SOURCE_ROOT}/qdldl-0.1.8" \
    -DOSQP_VERSION=1.0.0 \
    -DOSQP_BUILD_SHARED_LIB=OFF \
    -DOSQP_BUILD_STATIC_LIB=ON \
    -DOSQP_BUILD_DEMO_EXE=OFF \
    -DOSQP_BUILD_UNITTESTS=OFF \
    -DOSQP_CODEGEN=OFF \
    -DOSQP_ENABLE_DERIVATIVES=OFF
uv run --frozen cmake --build "${BUILD_ROOT}/osqp" --parallel "${BUILD_JOBS}"
uv run --frozen cmake --install "${BUILD_ROOT}/osqp"

uv run --frozen cmake \
    -S "${SOURCE_ROOT}/osqp-eigen-0.11.2" \
    -B "${BUILD_ROOT}/osqp-eigen" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${OSQP_PREFIX}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_PREFIX_PATH="${OSQP_PREFIX}" \
    -DCMAKE_TOOLCHAIN_FILE="${CONAN_OUTPUT_DIRECTORY}/conan_toolchain.cmake" \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTING=OFF \
    -DOSQP_IS_V1=ON \
    -DOSQP_IS_V1_FINAL=ON \
    -DOSQP_EIGEN_DEBUG_OUTPUT=OFF \
    -DOSQP_EIGEN_OSQP_TARGET_TO_LINK=osqp::osqpstatic
uv run --frozen cmake --build "${BUILD_ROOT}/osqp-eigen" --parallel "${BUILD_JOBS}"
uv run --frozen cmake --install "${BUILD_ROOT}/osqp-eigen"

printf '%s\n' "${OSQP_PREFIX}" > "${INSTALL_MARKER}"
echo "Installed OSQP stack to ${OSQP_PREFIX}"
