#!/bin/bash

# PYTHON PACKAGE ccu_bautiro
# CODE-GENERATED WITH pyecoregen
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

check_exit() {
    printf "pyecoregen 'ccu_bautiro': "
    if [[ $1 -ne 0 ]]; then
        printf "${RED}ERROR"
    else
        printf "${GREEN}SUCCESS"
    fi
    printf "${NC}\n"
}

# change into script-directory
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd $SCRIPTPATH

# clean >> codegen >> add *.ecore
rm -fr src/ccu_bautiro/*
pyecoregen -e model/ccu_bautiro.ecore -o src
cp model/ccu_bautiro.ecore src/ccu_bautiro/ccu_bautiro.ecore

check_exit $?
