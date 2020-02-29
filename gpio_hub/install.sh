#!/usr/bin/env bash

echo "Running gpio_hub install script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/gpio_hub

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/gpio_hub
fi
mkdir -p ${BASE_INSTALL_DIR}/logs

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}

echo "Installing python source code"
mkdir -p ${BASE_INSTALL_DIR}
rsync -a --delete ${SRC_DIR} ${BASE_INSTALL_DIR}

echo "Installing python dependencies"
# pip3 install -U -r ${SRC_DIR}/requirements.txt

chmod +x ./systemd/install_systemd.sh
./systemd/install_systemd.sh

echo "gpio_hub installation complete"
