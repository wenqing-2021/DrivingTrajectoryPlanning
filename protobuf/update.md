# Protobuf 更新与 `*.pyi` 生成说明

本文档用于记录：当你修改了 `protobuf/*.proto` 之后，如何更新 Python 类型提示文件 `src/protobuf/*_pb2.pyi`。

## 1. 环境依赖

在项目根目录执行以下命令，确保工具存在：

```bash
pip install protobuf==3.19.0
pip install mypy-protobuf==3.0.0
```

如果 `protoc` 不存在（`command not found`），先安装系统 protobuf 编译器。

## 2. 一键更新 pyi（推荐）

项目已提供脚本：`scripts/mypy_protobuf.sh`

```bash
bash scripts/mypy_protobuf.sh
```

脚本内部实际执行的是：

```bash
protoc --proto_path=protobuf --mypy_out=src/protobuf protobuf/*.proto
```

## 3. 检查是否更新成功

执行后应看到类似输出：

```text
Writing mypy to cost_map_pb2.pyi
Writing mypy to kinematic_model_pb2.pyi
Writing mypy to params_pb2.pyi
Writing mypy to problem_pb2.pyi
```

然后可用 git 查看变更：

```bash
git status
git diff src/protobuf/params_pb2.pyi
```

## 4. 常见问题

`protoc-gen-mypy: program not found or is not executable`
说明 `mypy-protobuf` 没安装到当前 Python 环境里，重新安装：

```bash
pip install mypy-protobuf==3.0.0
```

`protoc: command not found`
说明系统缺少 protobuf 编译器，需要先安装 `protoc`。

## 5. 备注

- `*.pyi` 是自动生成文件，不要手改。
- 每次改完 `protobuf/*.proto`，都建议重新执行一次脚本并提交变更。
