#!/usr/bin/env bash

echo "Running client uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/rover6_py

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/rover6_py
fi
rm -r ${BASE_INSTALL_DIR}
