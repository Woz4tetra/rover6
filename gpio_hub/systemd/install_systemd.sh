#!/usr/bin/env bash

echo "Running gpio_hub systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6/gpio_hub
fi

SCRIPT_NAME=gpio_hub
SERVICE_NAME=gpio_hub.service

chmod +x ${BASE_DIR}/${SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_DIR}/${SERVICE_NAME} ~/.config/systemd/user
cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user enable ${SERVICE_NAME}
echo "gpio_hub systemd service installation complete"
