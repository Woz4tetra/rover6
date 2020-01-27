#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0))")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi
mkdir -p ${BIN_INSTALL_DIR}
mkdir -p ${BIN_INSTALL_DIR}/logs

cp -r ${BASE_DIR}/config ${BIN_INSTALL_DIR}

pip3 install -U -r ${BASE_DIR}/requirements.txt
