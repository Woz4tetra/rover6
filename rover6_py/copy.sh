#!/usr/bin/env bash

echo "Running rover6_py copy script"

if [ "${BASE_DIR}" = "" ]; then
    BASE_DIR=$(realpath "$(dirname $0)")
fi
if [ "${SRC_DIR}" = "" ]; then
    SRC_DIR=${BASE_DIR}/rover6_py
fi

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=~/.local/rover6/rover6_py
fi
mkdir -p ${BASE_INSTALL_DIR}/logs
mkdir -p ${BASE_INSTALL_DIR}/data

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}
echo ${BASE_INSTALL_DIR}

echo "Installing python source code"
mkdir -p ${BASE_INSTALL_DIR}
rsync -a --delete ${SRC_DIR} ${BASE_INSTALL_DIR}

if systemctl --all --user | grep -Fq 'rover6_py'; then
    systemctl restart --user rover6_py.service
fi

echo "rover6_py copy complete"
