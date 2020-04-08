#!/usr/bin/env bash

echo "Running firmware install script"

BASE_DIR=$(realpath "$(dirname $0)")
if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=~/.local/bin
fi
SCRIPTS_DIR=${BASE_DIR}/scripts

mkdir -p ${BASE_INSTALL_DIR}

chmod +x ${SCRIPTS_DIR}/monitor-rover
chmod +x ${SCRIPTS_DIR}/upload-firmware
chmod +x ${SCRIPTS_DIR}/compile-firmware

cp ${SCRIPTS_DIR}/monitor-rover ${BASE_INSTALL_DIR}
cp ${SCRIPTS_DIR}/upload-firmware ${BASE_INSTALL_DIR}
cp ${SCRIPTS_DIR}/compile-firmware ${BASE_INSTALL_DIR}

if [ ! -f /etc/udev/rules.d/49-teensy.rules ]; then
  echo "Attempting to install platformio teensy rules"
  wget https://www.pjrc.com/teensy/49-teensy.rules -O /tmp/49-teensy.rules
  sudo mv /tmp/49-teensy.rules /etc/udev/rules.d/49-teensy.rules

  echo "Adding user '$USER' to the dialout group (USB permissions)"
  sudo usermod -a -G dialout $USER
fi

echo "Installing platformio"
pip3 install platformio
source ~/.profile
platformio --version

echo "Please close this terminal session to load platformio in ~/.local/bin"
echo "run 'upload-firmware' afterwards"
