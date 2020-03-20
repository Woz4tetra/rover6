#!/usr/bin/env bash

echo "Running rover6_py install script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/rover6_py

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/rover6_py
fi
mkdir -p ${BASE_INSTALL_DIR}/logs
mkdir -p ${BASE_INSTALL_DIR}/data

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}
echo ${BASE_INSTALL_DIR}

echo "Installing python source code"
mkdir -p ${BASE_INSTALL_DIR}
rsync -a --delete ${SRC_DIR} ${BASE_INSTALL_DIR}

echo "Installing python dependencies"
# pip3 install -U -r ${SRC_DIR}/requirements.txt
# sudo apt-get install libatlas-base-dev  # for numpy

./systemd/install_systemd.sh

echo "rover6_py installation complete"
