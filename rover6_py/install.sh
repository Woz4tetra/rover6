#!/usr/bin/env bash

echo "Running client install script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/rover6_py

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/rover6_py
fi
mkdir -p ${BASE_INSTALL_DIR}/logs

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}

echo "Installing python source code"
mkdir -p ${BASE_INSTALL_DIR}/rover6_py
rsync -a --delete ${SRC_DIR} ${BASE_INSTALL_DIR}/rover6_py

echo "Installing python dependencies"
pip3 install -U -r ${SRC_DIR}/requirements.txt
echo "client installation complete"
