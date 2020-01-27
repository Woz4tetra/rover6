#!/usr/bin/env bash

echo "Running systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

chmod +x ${BASE_DIR}/rover6.sh

echo ${BASE_INSTALL_DIR}/bin

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_DIR}/rover6.service ~/.config/systemd/user
cp ${BASE_DIR}/rover6.sh ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart rover6.service
loginctl enable-linger $USER
systemctl --user enable rover6.service
echo "systemd service installation complete"
