#!/usr/bin/env bash

echo "Running service install scripts"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

# bash rover6_py/install_rover6_py.sh
bash rover6_ros/install_rover6_ros.sh

echo "service installation complete"
