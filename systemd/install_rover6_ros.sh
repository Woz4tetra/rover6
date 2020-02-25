#!/usr/bin/env bash

echo "Running ros systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

SCRIPT_NAME=rover6_ros
SERVICE_NAME=rover6_ros.service
BASE_SCRIPT_DIR=${BASE_DIR}/${SCRIPT_NAME}

chmod +x ${BASE_SCRIPT_DIR}/${SCRIPT_NAME}

echo "Copying scripts"
BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}
cp ${BASE_SCRIPT_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_SCRIPT_DIR}/rover6_ros_restart ${BIN_INSTALL_DIR}
cp ${BASE_SCRIPT_DIR}/rover6_ros_stop ${BIN_INSTALL_DIR}
cp ${BASE_SCRIPT_DIR}/rover6_ros_tail ${BIN_INSTALL_DIR}


echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_SCRIPT_DIR}/${SERVICE_NAME} ~/.config/systemd/user

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user enable ${SERVICE_NAME}
echo "ros systemd service installation complete"
