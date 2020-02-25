#!/usr/bin/env bash

echo "Running ros systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

SCRIPT_NAME=rover6_py
SERVICE_NAME=rover6_py.service
BASE_SCRIPT_DIR=${BASE_DIR}/${SCRIPT_NAME}

chmod +x ${BASE_SCRIPT_DIR}/${SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_SCRIPT_DIR}/${SERVICE_NAME} ~/.config/systemd/user
cp ${BASE_SCRIPT_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user enable ${SERVICE_NAME}
echo "ros systemd service installation complete"
