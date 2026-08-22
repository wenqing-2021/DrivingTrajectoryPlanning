#!/usr/bin/env bash
# Install the project's recommended VS Code extensions by downloading the
# VSIX packages with curl and installing the local files.
set -euo pipefail

EXTENSIONS=(
    "ms-vscode.cpptools-extension-pack"
    "llvm-vs-code-extensions.vscode-clangd"
    "eamodio.gitlens"
    "mhutchie.git-graph"
    "ms-python.python"
    "ms-python.black-formatter"
    "charliermarsh.ruff"
)

MARKETPLACE_URL="https://marketplace.visualstudio.com/_apis/public/gallery/publishers"

if ! command -v code >/dev/null 2>&1; then
    echo "VS Code CLI 'code' is unavailable in PATH." >&2
    exit 1
fi
if ! command -v curl >/dev/null 2>&1; then
    echo "'curl' is unavailable in PATH." >&2
    exit 1
fi

download_dir="$(mktemp -d)"
trap 'rm -rf "${download_dir}"' EXIT

# Download one extension's VSIX package from the marketplace and print the
# local file path on success.
download_vsix() {
    local extension="$1"
    local publisher="${extension%%.*}"
    local name="${extension#*.}"
    local download_file="${download_dir}/${extension}.download"
    local vsix_file="${download_dir}/${extension}.vsix"
    local url="${MARKETPLACE_URL}/${publisher}/vsextensions/${name}/latest/vspackage"

    echo "Downloading ${extension} from ${url}" >&2
    if ! curl -fsSL --retry 3 --connect-timeout 30 \
        --output "${download_file}" "${url}"; then
        echo "Failed to download ${extension}" >&2
        return 1
    fi

    # The marketplace may serve the package gzip-compressed, while a VSIX is
    # a ZIP archive; decompress when the gzip magic bytes are present.
    local magic
    magic="$(head --bytes 2 "${download_file}" | od -An -tx1 | tr -d ' \n')"
    if [[ "${magic}" == "1f8b" ]]; then
        if ! gzip --decompress --stdout "${download_file}" > "${vsix_file}"; then
            echo "Failed to decompress the package of ${extension}" >&2
            return 1
        fi
    else
        mv "${download_file}" "${vsix_file}"
    fi

    magic="$(head --bytes 2 "${vsix_file}" | od -An -tx1 | tr -d ' \n')"
    if [[ "${magic}" != "504b" ]]; then
        echo "The downloaded package of ${extension} is not a valid VSIX file" >&2
        return 1
    fi

    printf '%s\n' "${vsix_file}"
}

# The VS Code CLI may exit 0 even when an installation fails, so the
# output has to be checked as well as the exit status.
failed_extensions=()
for extension in "${EXTENSIONS[@]}"; do
    download_status=0
    vsix_file="$(download_vsix "${extension}")" || download_status=$?
    if (( download_status != 0 )); then
        failed_extensions+=("${extension}")
        continue
    fi

    install_status=0
    install_output="$(code --install-extension "${vsix_file}" 2>&1)" || install_status=$?
    printf '%s\n' "${install_output}"
    if (( install_status != 0 )) \
        || grep -q "Failed Installing Extensions" <<< "${install_output}"; then
        failed_extensions+=("${extension}")
    fi
done

if (( ${#failed_extensions[@]} > 0 )); then
    echo "Failed to install extensions: ${failed_extensions[*]}" >&2
    exit 1
fi
echo "All ${#EXTENSIONS[@]} extensions are installed."
