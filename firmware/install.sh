#!/usr/bin/env bash

echo "Running firmware install script"

BASE_DIR=$(realpath "$(dirname $0)")
if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

mkdir -p ${BASE_INSTALL_DIR}/bin

chmod +x ${BASE_DIR}/monitor-rover
chmod +x ${BASE_DIR}/upload-rover

cp ${BASE_DIR}/monitor-rover ${BASE_INSTALL_DIR}/bin
cp ${BASE_DIR}/upload-rover ${BASE_INSTALL_DIR}/bin

ln -s ${BASE_INSTALL_DIR}/bin/monitor-rover ~/.local/bin
ln -s ${BASE_INSTALL_DIR}/bin/upload-rover ~/.local/bin

if [ ! -f /etc/udev/rules.d/49-teensy.rules ]; then
  echo "Attempting to install platformio teensy rules"
  wget https://www.pjrc.com/teensy/49-teensy.rules -O /tmp/49-teensy.rules
  sudo mv /tmp/49-teensy.rules /etc/udev/rules.d/49-teensy.rules
fi

echo "Adding user '$USER' to the dialout group (USB permissions)"
sudo usermod -a -G dialout $USER

echo "Installing platformio"
pip3 install platformio
source ~/.profile
platformio --version

echo "Uploading firmware to teensy device"
upload-rover
echo "firmware installation complete"
