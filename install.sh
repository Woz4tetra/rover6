#!/usr/bin/env bash

echo "Running rover6 install"

BASE_DIR=$(realpath "$(dirname $0)")
BASE_INSTALL_DIR=~/.local/rover6

mkdir -p ${BASE_INSTALL_DIR}

if [ "${BASE_DIR}" != "$HOME/rover6" ]; then
  cp -r "${BASE_DIR}" "$HOME/rover6"
fi

bash ${BASE_DIR}/config/install.sh
# bash ${BASE_DIR}/firmware/install.sh
# bash ${BASE_DIR}/rover6_py/install.sh
# bash ${BASE_DIR}/rover6_ros/install.sh
