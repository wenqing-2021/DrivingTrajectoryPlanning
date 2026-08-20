"""Install the project's recommended VS Code extensions."""

from __future__ import annotations

import argparse
import shutil
import subprocess

EXTENSIONS = (
    "ms-vscode.cpptools",
    "llvm-vs-code-extensions.vscode-clangd",
    "ms-vscode.cmake-tools",
    "eamodio.gitlens",
    "mhutchie.git-graph",
    "ms-python.python",
    "ms-python.black-formatter",
    "charliermarsh.ruff",
    "GitHub.copilot",
)


def install_extensions() -> None:
    """Install each recommended extension with the VS Code CLI."""
    if shutil.which("code") is None:
        raise SystemExit("VS Code CLI 'code' is unavailable in PATH.")
    for extension in EXTENSIONS:
        subprocess.run(
            ["code", "--install-extension", extension],
            check=True,
        )


def main() -> None:
    """Parse CLI arguments and install extensions."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", "-i", action="store_true")
    args = parser.parse_args()
    if not args.install:
        parser.error("use --install to install the extension list")
    install_extensions()


if __name__ == "__main__":
    main()
