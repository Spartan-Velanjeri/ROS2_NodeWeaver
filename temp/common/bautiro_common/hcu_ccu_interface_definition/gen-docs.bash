#!/bin/sh

  docker run --rm \
    -v "$(pwd)/doc:/out" \
    -v "$(pwd)/protos:/protos" \
    -v "$(pwd)/entrypoint.sh:/entrypoint.sh" \
    pseudomuto/protoc-gen-doc
