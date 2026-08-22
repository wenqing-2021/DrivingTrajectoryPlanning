#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
EIGENGDB_DIRECTORY="${REPOSITORY_ROOT}/third-party/EigenGdb"
GDB_INIT_DIRECTORY="${HOME}/.gdbinit.d"
GDB_INIT_FILE="${HOME}/.gdbinit"

if [[ ! -f "${EIGENGDB_DIRECTORY}/printers.py" ]]; then
    echo "EigenGdb submodule is missing under third-party/EigenGdb." >&2
    echo "Clone it with: git submodule update --init -- third-party/EigenGdb" >&2
    exit 1
fi

mkdir -p "${GDB_INIT_DIRECTORY}"
cp "${EIGENGDB_DIRECTORY}/printers.py" "${GDB_INIT_DIRECTORY}/printers.py"
cp "${EIGENGDB_DIRECTORY}/__init__.py" "${GDB_INIT_DIRECTORY}/__init__.py"

if [[ -f "${GDB_INIT_FILE}" ]] && grep -Fq "register_eigen_printers" "${GDB_INIT_FILE}"; then
    echo "Eigen gdb printers are already registered in ${GDB_INIT_FILE}."
    exit 0
fi

cat >> "${GDB_INIT_FILE}" <<EOF

# Eigen gdb pretty-printers (third-party/EigenGdb)
python
import sys
sys.path.insert(0, '${GDB_INIT_DIRECTORY}')
from printers import register_eigen_printers
register_eigen_printers(None)
end
EOF

echo "Registered Eigen gdb printers in ${GDB_INIT_FILE}."
