#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"

export UV_CACHE_DIR="${UV_CACHE_DIR:-${REPOSITORY_ROOT}/.cache/uv}"

if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
fi

if command -v apt-get >/dev/null 2>&1; then
    system_packages=(
        autoconf
        automake
        build-essential
        cppad
        gfortran
        git
        libblas-dev
        liblapack-dev
        libmetis-dev
        libtool
        patch
        pkg-config
        python3-dev
        wget
    )
    missing_packages=()
    for package in "${system_packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null \
            | grep -q "ok installed"; then
            missing_packages+=("${package}")
        fi
    done

    if (( ${#missing_packages[@]} > 0 )); then
        echo "Installing system toolchain: ${missing_packages[*]}"
        if (( EUID == 0 )); then
            apt-get update
            apt-get install -y "${missing_packages[@]}"
        elif command -v sudo >/dev/null 2>&1; then
            sudo apt-get update
            sudo apt-get install -y "${missing_packages[@]}"
        else
            echo "Install the missing packages above, then rerun this script." >&2
            exit 1
        fi
    fi
fi

cd "${REPOSITORY_ROOT}"
uv_sync_arguments=(sync --all-groups --frozen)
if [[ -f "${REPOSITORY_ROOT}/.venv/bin/cmake" ]] \
    && ! head -n 1 "${REPOSITORY_ROOT}/.venv/bin/cmake" \
        | grep -Fq "${REPOSITORY_ROOT}/.venv/bin/python"; then
    uv_sync_arguments+=(--reinstall)
fi
uv "${uv_sync_arguments[@]}"
bash "${SCRIPT_DIRECTORY}/install_conan.sh"
bash "${SCRIPT_DIRECTORY}/install_ipopt.sh"
bash "${SCRIPT_DIRECTORY}/install_osqp.sh"
bash "${SCRIPT_DIRECTORY}/install_eigengdb.sh"
exec bash "${SCRIPT_DIRECTORY}/build.sh"
