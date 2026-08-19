#!/bin/bash

# Format first-party Python and C++ files.
black src tests data/BenchmarkCases/RunMe.py

if command -v clang-format >/dev/null 2>&1; then
    find src protobuf \
        -type f \( -name '*.cc' -o -name '*.cpp' -o -name '*.h' \) \
        -not -path '*/third-party/*' \
        -exec clang-format -i {} +
fi
