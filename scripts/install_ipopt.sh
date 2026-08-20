#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
SOURCE_ROOT="${REPOSITORY_ROOT}/.cache/ipopt/sources"
BUILD_ROOT="${REPOSITORY_ROOT}/.cache/ipopt/build"
IPOPT_PREFIX="${IPOPT_PREFIX:-${REPOSITORY_ROOT}/.local/ipopt}"
BUILD_JOBS="${BUILD_JOBS:-4}"
COINHSL_ARCHIVE="${REPOSITORY_ROOT}/third-party/coinhsl.zip"
COINHSL_SHA256="91f53d3fce5fcb8ab25e4ca8e94e9b79bf7febd070639a7535c3fc09c39d628d"

# Immutable revisions resolved from the upstream repositories.
ASL_REVISION="3cada39b85d3073d6d49b9a527bf7b98d5001b2c"
HSL_REVISION="ec509dcf102525216985d4cf8b61996028bc155d"
MUMPS_REVISION="74415ebfd072bc06ec9ac3f16b222af85580011a"
IPOPT_REVISION="2695946fa79d2e84f3034e065e788933a81466eb"

export PKG_CONFIG_PATH="${IPOPT_PREFIX}/lib/pkgconfig:${IPOPT_PREFIX}/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="${IPOPT_PREFIX}/lib:${IPOPT_PREFIX}/lib64:${LD_LIBRARY_PATH:-}"

HSL_MODE="without-hsl"
if [[ -f "${COINHSL_ARCHIVE}" ]]; then
    if [[ "$(sha256sum "${COINHSL_ARCHIVE}" | cut -d ' ' -f 1)" != "${COINHSL_SHA256}" ]]; then
        echo "Coin-HSL archive checksum does not match the expected licensed asset." >&2
        exit 1
    fi
    HSL_MODE="with-hsl"
    echo "Licensed Coin-HSL archive detected; HSL support will be enabled."
else
    echo "No Coin-HSL archive detected; Ipopt will be built with MUMPS and without HSL."
fi

IPOPT_INSTALL_REVISION="${IPOPT_REVISION}-${HSL_MODE}"
IPOPT_INSTALL_MARKER="${IPOPT_PREFIX}/.installed-ipopt-${IPOPT_INSTALL_REVISION}"
required_ipopt_packages=(ipopt coinasl coinmumps)
if [[ "${HSL_MODE}" == "with-hsl" ]]; then
    required_ipopt_packages+=(coinhsl)
fi

for command_name in gcc g++ gfortran git make pkg-config python3; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Missing Ipopt build tool: ${command_name}" >&2
        echo "Run 'bash scripts/setup.sh' to install host prerequisites." >&2
        exit 1
    fi
done

installed_ipopt_prefix="$(pkg-config --variable=prefix ipopt 2>/dev/null || true)"
installed_ipopt_marker_prefix=""
if [[ -f "${IPOPT_INSTALL_MARKER}" ]]; then
    IFS= read -r installed_ipopt_marker_prefix < "${IPOPT_INSTALL_MARKER}" || true
fi
if [[ "${installed_ipopt_prefix}" == "${IPOPT_PREFIX}" ]] \
    && [[ "${installed_ipopt_marker_prefix}" == "${IPOPT_PREFIX}" ]] \
    && pkg-config --exists "${required_ipopt_packages[@]}"; then
    echo "Ipopt and its required ASL/MUMPS dependencies are already installed (${HSL_MODE})."
    exit 0
fi

mkdir -p "${SOURCE_ROOT}" "${BUILD_ROOT}" "${IPOPT_PREFIX}"

fetch_source() {
    local name="$1"
    local url="$2"
    local revision="$3"
    local source_directory="${SOURCE_ROOT}/${name}"

    if [[ ! -d "${source_directory}/.git" ]]; then
        git clone --filter=blob:none --no-checkout "${url}" "${source_directory}"
    fi
    git -C "${source_directory}" fetch --depth 1 origin "${revision}"
    git -C "${source_directory}" checkout --detach FETCH_HEAD
}

configure_and_install() {
    local name="$1"
    local source_directory="$2"
    local revision="$3"
    shift 3
    local build_directory="${BUILD_ROOT}/${name}"
    local marker="${IPOPT_PREFIX}/.installed-${name}-${revision}"
    local build_jobs="${BUILD_JOBS}"
    local marker_prefix=""

    if [[ -f "${marker}" ]]; then
        IFS= read -r marker_prefix < "${marker}" || true
    fi
    if [[ "${marker_prefix}" == "${IPOPT_PREFIX}" ]]; then
        return
    fi
    # Autotools dependency files contain absolute source paths.  Reusing a
    # generated build tree after the repository is moved leaves those paths
    # pointing at the previous checkout location.
    rm -rf "${build_directory}"
    # ThirdParty-Mumps does not declare every Fortran module dependency in its
    # generated Makefile. Parallel compilation can therefore consume a .mod
    # file before its producer has finished.
    if [[ "${name}" == "mumps" ]]; then
        build_jobs=1
    fi
    mkdir -p "${build_directory}"
    cd "${build_directory}"
    "${source_directory}/configure" --prefix="${IPOPT_PREFIX}" "$@"
    make --jobs "${build_jobs}"
    if [[ "${name}" == "ipopt" ]]; then
        make test
    fi
    make install
    printf '%s\n' "${IPOPT_PREFIX}" > "${marker}"
}

fetch_source \
    ThirdParty-ASL \
    https://github.com/coin-or-tools/ThirdParty-ASL.git \
    "${ASL_REVISION}"
cd "${SOURCE_ROOT}/ThirdParty-ASL"
if [[ ! -d solvers ]]; then
    ./get.ASL
fi
configure_and_install \
    asl "${SOURCE_ROOT}/ThirdParty-ASL" "${ASL_REVISION}"

if [[ "${HSL_MODE}" == "with-hsl" ]]; then
    fetch_source \
        ThirdParty-HSL \
        https://github.com/coin-or-tools/ThirdParty-HSL.git \
        "${HSL_REVISION}"
    if [[ ! -d "${SOURCE_ROOT}/ThirdParty-HSL/coinhsl" ]]; then
        python3 -m zipfile -e \
            "${COINHSL_ARCHIVE}" \
            "${SOURCE_ROOT}/ThirdParty-HSL"
    fi
    configure_and_install \
        hsl "${SOURCE_ROOT}/ThirdParty-HSL" "${HSL_REVISION}"
fi

fetch_source \
    ThirdParty-Mumps \
    https://github.com/coin-or-tools/ThirdParty-Mumps.git \
    "${MUMPS_REVISION}"
cd "${SOURCE_ROOT}/ThirdParty-Mumps"
if [[ ! -d MUMPS ]]; then
    ./get.Mumps
fi
configure_and_install \
    mumps "${SOURCE_ROOT}/ThirdParty-Mumps" "${MUMPS_REVISION}"

fetch_source \
    Ipopt \
    https://github.com/coin-or/Ipopt.git \
    "${IPOPT_REVISION}"
ipopt_configure_options=(--disable-java)
if [[ "${HSL_MODE}" == "with-hsl" ]]; then
    ipopt_configure_options+=(--with-hsl)
else
    ipopt_configure_options+=(--without-hsl)
fi
configure_and_install \
    ipopt "${SOURCE_ROOT}/Ipopt" "${IPOPT_INSTALL_REVISION}" \
    "${ipopt_configure_options[@]}"

pkg-config --modversion ipopt
