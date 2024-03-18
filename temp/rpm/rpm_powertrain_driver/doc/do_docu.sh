#!/bin/bash
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
MNT="/docs" # volume name inside docker-container for spinx documents

SPHINX_LOCAL_IMAGE_NAME="sphinx_local"
OUT_FOLDER_NAME="__OUTPUT$2" # .gitignore

DOC=$(dirname "$(readlink -f "$0")")
cd $DOC
if [[ "$(docker images -q $SPHINX_LOCAL_IMAGE_NAME 2>/dev/null)" == "" ]]; then
    if [ ! -f "$DOC/Dockerfile" ]; then
        echo "FROM sphinxdoc/sphinx" >Dockerfile
        echo "USER root" >>Dockerfile
        echo "RUN  pip install --upgrade pip && pip install --upgrade myst-parser sphinx-autobuild sphinx-rtd-theme" >>Dockerfile
        echo "ENV  PYTHONDONTWRITEBYTECODE=1" >>Dockerfile
    fi
    docker build -t $SPHINX_LOCAL_IMAGE_NAME .
fi

if [ -f "docubuild-repo-structure.yml" ]; then
    txt="${RED}conf.py${NC}" 
    ARGs=""
    WS=$(dirname "$(dirname "$DOC")")
    SRC_PATH="$MNT/$(basename $(dirname "$DOC"))/$(basename $DOC)"
    DST_PATH="$SRC_PATH/$OUT_FOLDER_NAME"
else
    txt="${RED}flat${NC}"
    ARGS="-C -D exclude_patterns=**/readme.md -D extensions=myst_parser -D html_theme=sphinx_rtd_theme"
    WS=$DOC
    SRC_PATH=$MNT
    DST_PATH="$MNT/$OUT_FOLDER_NAME"

fi
CMD="docker run -u $UID:$UID --rm -it -v $WS:$MNT $SPHINX_LOCAL_IMAGE_NAME sphinx-build $1 -Ea $ARGS $SRC_PATH $DST_PATH"
printf "$txt sphinx-build in ${YELLOW}$WS${NC} output at: ${YELLOW}$OUT_FOLDER_NAME${NC} via:\n"
printf "${CYAN}$CMD${NC}\n" 
$CMD
