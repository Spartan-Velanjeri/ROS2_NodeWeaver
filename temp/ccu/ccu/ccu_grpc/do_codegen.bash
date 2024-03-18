#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# most important configurable
PROTO_SOURCES_CONFIG=(../../bautiro_common/hcu_ccu_interface_definition/protos
  ../../../common/bautiro_common/hcu_ccu_interface_definition/protos)

get_first_folder() {
  for folder in "${PROTO_SOURCES_CONFIG[@]}"; do
    if [ -d "$folder" ]; then
      echo "$folder"
      return
    fi
  done
  return 1
}

###################################################################################################
#
#
#
###################################################################################################

# generate Python-Code for gRPC
PROTO_FOLDER=$(get_first_folder)
if [ $? -eq 0 ]; then
  printf "${GREEN}Found Proto at${NC} $PROTO_FOLDER\n"
else
  printf "${RED}Not found at${NC}\n"
  for folder in "${PROTO_SOURCES_CONFIG[@]}"; do
    printf "    $folder\n"
  done
  printf "${RED}--> early exit${NC}\n"
  exit 1
fi

# Holt die aktuelle Commit-ID und speichert sie in einer Variablen
COMMIT_ID=$(git -C "$PROTO_FOLDER" rev-parse HEAD)
[ $? -ne 0 ] && printf "${RED}ERROR${NC}: could not retrieve commit-id." && exit 1
BRANCH_NAME=$(git -C "$PROTO_FOLDER" rev-parse --abbrev-ref HEAD)
[ $? -ne 0 ] && printf "${RED}ERROR${NC}: could not retrieve branch name." && exit 1

# PYTHON PACKAGE Cgrpc
# CODE-GENERATED WITH grpc_tools

# change into script-directory
CODEGEN_SCRIPT=$(readlink -f "$0")
PROJECT_PATH=$(dirname "$CODEGEN_SCRIPT")
cd $PROJECT_PATH

# take over the PACKAGE_NAME from (first) folder in src
PACKAGE_NAME=$(ls -1 "${PROJECT_PATH}/src" | head -n 1)
check_exit() {
  printf "grpc-proto-codegen: '$PACKAGE_NAME': "
  if [[ $1 -ne 0 ]]; then
    printf "${RED}ERROR"
  else
    printf "${GREEN}SUCCESS"
  fi
  printf "${NC}\n"
}

GEN_FOLDER=src/$PACKAGE_NAME

# swipe away old code
rm -fr $GEN_FOLDER
mkdir -p $GEN_FOLDER

# Iterate over each .proto file in the PROTO_FOLDER and its subdirectories
for PROTO_FILE in $(find $PROTO_FOLDER -name '*.proto'); do
  python3 -m grpc_tools.protoc \
    -I$PROTO_FOLDER \
    --python_out=$GEN_FOLDER \
    --pyi_out=$GEN_FOLDER \
    --grpc_python_out=$GEN_FOLDER \
    $PROTO_FILE
  check_exit $?
done

# make each (sub)folder a py-package
find $GEN_FOLDER -type d -exec touch {}/__init__.py \;

### unfortunately we touch the generated file
#   convert import statements       'from                msg import'
#   (skip google and typing)        'from <PACKAGE_NAME>.msg import'
sed -i \
  -e "/from google\|from typing\|from grpc/!s|^from \([^ ]*\)|from ${PACKAGE_NAME}.\1|g" \
  -e "/import google\|import typing\|import grpc/!s|^import \([^ ]*\)|import ${PACKAGE_NAME}.\1|g" \
  $GEN_FOLDER/*.p*

# import .common_types_pb2 as common__types__pb2

#
# add util.py to package
#
cp util.py $GEN_FOLDER/util.py

# write COMMIT-ID into top-level __init__py.py
echo "# Source ($PROTO_FOLDER) commit $COMMIT_ID" >$GEN_FOLDER/__init__.py
echo "# Source ($PROTO_FOLDER) branch $BRANCH_NAME" >>$GEN_FOLDER/__init__.py
