#!/bin/sh

PROTO_FILES=$(find /protos -name '*.proto')

echo "========= PROTO FILES ==========="
echo $PROTO_FILES
echo "================================="

rm -r /out/*

protoc -I /protos $PROTO_FILES --doc_out=/out \
    --doc_opt=markdown,docs.md

protoc -I /protos $PROTO_FILES --doc_out=/out \
    --doc_opt=html,index.html

protoc -I /protos $PROTO_FILES --doc_out=/out \
    --doc_opt=docbook,doc.dbk

protoc -I /protos $PROTO_FILES --doc_out=/out \
    --doc_opt=json,doc.json

echo "============ DONE ==============="
