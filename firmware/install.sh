#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0))")
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

pip install platformio
platformio -v

upload-rover
