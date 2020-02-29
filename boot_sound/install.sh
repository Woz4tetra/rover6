#!/usr/bin/env bash

echo "Running boot sound systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")
SOUND_FILE=boot_sound.wav

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/boot_sound
fi

chmod +x ${BASE_DIR}/boot_sound.sh

echo ${BASE_INSTALL_DIR}/bin

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_DIR}/boot_sound.service ~/.config/systemd/user
cp ${BASE_DIR}/boot_sound.sh ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${SOUND_FILE} ${BASE_INSTALL_DIR}

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart boot_sound.service
loginctl enable-linger $USER
systemctl --user enable boot_sound.service
echo "boot_sound systemd service installation complete"
