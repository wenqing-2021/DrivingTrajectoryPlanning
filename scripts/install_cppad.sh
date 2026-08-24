#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
SOURCE_DIRECTORY="${REPOSITORY_ROOT}/.cache/cppad/sources/CppAD"
BUILD_DIRECTORY="${REPOSITORY_ROOT}/.cache/cppad/build"
CPPAD_PREFIX="${CPPAD_PREFIX:-${REPOSITORY_ROOT}/.local/cppad}"
CPPAD_VERSION="20210000.8"
CPPAD_REVISION="bd3d6dc866a8cb4f0fe5ecf6ab0955391412af0d"
BUILD_JOBS="${BUILD_JOBS:-4}"
INSTALL_MARKER="${CPPAD_PREFIX}/.installed-cppad-${CPPAD_VERSION}"
CALLBACK_HEADER="${CPPAD_PREFIX}/include/cppad/ipopt/solve_callback.hpp"
CONFIGURE_HEADER="${CPPAD_PREFIX}/include/cppad/configure.hpp"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"

for command_name in g++ git uv; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Missing CppAD build tool: ${command_name}" >&2
        echo "Run 'bash scripts/setup.sh' to install host prerequisites." >&2
        exit 1
    fi
done

installed_prefix=""
if [[ -f "${INSTALL_MARKER}" ]]; then
    IFS= read -r installed_prefix < "${INSTALL_MARKER}" || true
fi
if [[ "${installed_prefix}" == "${CPPAD_PREFIX}" ]] \
    && [[ -f "${CPPAD_PREFIX}/include/cppad/cppad.hpp" ]] \
    && grep -Fq '<coin-or/IpIpoptApplication.hpp>' "${CALLBACK_HEADER}" \
    && grep -Fq "CPPAD_PACKAGE_STRING \"cppad-${CPPAD_VERSION}\"" "${CONFIGURE_HEADER}"; then
    echo "CppAD ${CPPAD_VERSION} is already installed."
    exit 0
fi

mkdir -p "$(dirname "${SOURCE_DIRECTORY}")"
if [[ ! -d "${SOURCE_DIRECTORY}/.git" ]]; then
    git clone --filter=blob:none --no-checkout \
        https://github.com/coin-or/CppAD.git "${SOURCE_DIRECTORY}"
fi
git -C "${SOURCE_DIRECTORY}" fetch --depth 1 origin "${CPPAD_REVISION}"
git -C "${SOURCE_DIRECTORY}" checkout --detach FETCH_HEAD

uv run --frozen --no-sync cmake -E remove_directory "${BUILD_DIRECTORY}"
uv run --frozen --no-sync cmake -E remove_directory "${CPPAD_PREFIX}"

uv run --frozen --no-sync cmake \
    -S "${SOURCE_DIRECTORY}" \
    -B "${BUILD_DIRECTORY}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -Dcppad_prefix="${CPPAD_PREFIX}"
uv run --frozen --no-sync cmake --build "${BUILD_DIRECTORY}" --parallel "${BUILD_JOBS}"
uv run --frozen --no-sync cmake --install "${BUILD_DIRECTORY}"

if ! grep -Fq '<coin-or/IpIpoptApplication.hpp>' "${CALLBACK_HEADER}"; then
    echo "Installed CppAD does not use the coin-or Ipopt include layout." >&2
    exit 1
fi
if ! grep -Fq "CPPAD_PACKAGE_STRING \"cppad-${CPPAD_VERSION}\"" "${CONFIGURE_HEADER}"; then
    echo "Installed CppAD version does not match ${CPPAD_VERSION}." >&2
    exit 1
fi

printf '%s\n' "${CPPAD_PREFIX}" > "${INSTALL_MARKER}"
echo "Installed CppAD ${CPPAD_VERSION} to ${CPPAD_PREFIX}"
