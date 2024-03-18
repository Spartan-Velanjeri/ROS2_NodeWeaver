# Cgrpc (Python Package)

> **``! GENERATED CODE !``**
>
> This python package is generated code
>
> **``DON'T DO MANUAL CHANGES``**

## gRPC - how to codegen - short

```bash
# run codegen
ccu_data_services/do_codegen.bash
```

## gRPC - how to codegen - manual way / details

```bash
cd <folder_where_your_proto-files_are>
python3 -m grpc_tools.protoc -I. --python_out=. --pyi_out=. --grpc_python_out=. example.proto
```

## gRPC - how to install

```bash
# Install gRPC:
$ pip install grpcio
# install code-gen
pip install grpcio-tools
```
