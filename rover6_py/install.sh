#!/usr/bin/env bash

echo "Running client install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi
mkdir -p ${BASE_INSTALL_DIR}
mkdir -p ${BASE_INSTALL_DIR}/logs

cp -r ${BASE_DIR}/config ${BASE_INSTALL_DIR}

echo "Installing python dependencies"
pip3 install -U -r ${BASE_DIR}/requirements.txt
echo "client installation complete"
