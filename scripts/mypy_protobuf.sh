#!/bin/bash
# if you update protobuf files, you need to run this script to generate mypy stubs

# # 1. Install mypy-protobuf
# pip install protobuf==3.19.0
# pip install mypy-protobuf==3.0.0

# 2. check there is protobuf file in the src directory
cd src
if [ ! -d "protobuf" ]; then
    echo "src directory does not exist"
    mkdir protobuf
    echo "src/protobuf directory created"
fi
cd ..
protoc --proto_path=protobuf --mypy_out=src/protobuf protobuf/*.proto 