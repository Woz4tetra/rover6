#!/usr/bin/env bash

echo "Running rover6_py install script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/rover6_py

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/rover6_py
fi

chmod +x ./copy.sh
./copy.sh

echo "Installing python dependencies"
# pip3 install -U -r ${SRC_DIR}/requirements.txt
# sudo apt-get install libatlas-base-dev  # for numpy

./systemd/install_systemd.sh

echo "rover6_py installation complete"
