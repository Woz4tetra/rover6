#!/usr/bin/env bash

echo "Running autohotspot service install script"

BASE_DIR=$(realpath "$(dirname $0)")

# install dependencies
#sudo apt-get update
#sudo apt-get upgrade
#sudo apt-get install hostapd dnsmasq   --assume-yes

# disable hostapd and dnsmasq. The script will launch it
sudo systemctl unmask hostapd
sudo systemctl disable hostapd
sudo systemctl disable dnsmasq

if [ -z "$ROVER6_HOSTAP_NAME" ]; then
    echo "ROVER6_HOSTAP_NAME=rover6_`hostname`" | sudo tee -a /etc/environment
fi

copy_and_backup () {
    local DEST=$1
    local OLD=$2
    local SRC=${BASE_DIR}/$3
    if [ -f ${DEST} ]; then
        if [ ! -f ${OLD} ]; then
            echo "Backing up ${DEST}"
            sudo mv ${DEST} ${OLD}
        fi
    fi
    sudo cp ${SRC} ${DEST}
}

echo "Installing hostapd config"
HOSTAPD_CONF_DEST=/etc/hostapd/hostapd.conf
HOSTAPD_CONF_OLD=/etc/hostapd/hostapd.conf.old
HOSTAPD_CONF_TMP=/tmp/hostapd.conf
HOSTAPD_CONF_SRC=${BASE_DIR}/hostapd.conf
if [ -f ${HOSTAPD_CONF_DEST} ]; then
    if [ ! -f ${HOSTAPD_CONF_OLD} ]; then
        echo "Backing up default hostapd config"
        sudo mv ${HOSTAPD_CONF_DEST} ${HOSTAPD_CONF_OLD}
    fi
fi
sed "s/%%ROVER6_SSID%%/${ROVER6_HOSTAP_NAME}/g" ${HOSTAPD_CONF_SRC} > ${HOSTAPD_CONF_TMP}
sudo mv ${HOSTAPD_CONF_TMP} ${HOSTAPD_CONF_DEST}

copy_and_backup /etc/default/hostapd /etc/default/hostapd.old hostapd

echo "Installing dnsmasq config"
copy_and_backup /etc/dnsmasq.conf /etc/dnsmasq.conf.old dnsmasq.conf

echo "Installing network interfaces config"
copy_and_backup /etc/network/interfaces /etc/network/interfaces.old interfaces

echo "Installing dhcpcd config"
copy_and_backup /etc/dhcpcd.conf /etc/dhcpcd.conf.old dhcpcd.conf

echo "Installing systemd service"
SERVICE_NAME=autohotspot.service
SERVICE_DEST=/etc/systemd/system/${SERVICE_NAME}
sudo cp ${BASE_DIR}/${SERVICE_NAME} ${SERVICE_DEST}

echo "Installing autohotspot script"
BIN_INSTALL_DIR=/usr/bin
SCRIPT_NAME=autohotspot
BIN_INSTALL_DEST=${BIN_INSTALL_DIR}/${SCRIPT_NAME}
sudo cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DEST}
echo ${BIN_INSTALL_DEST}

echo "Installing cron job"
CRON_JOB_NAME=autohotspot_cron
CRON_JOB_DEST=/etc/cron.d/${CRON_JOB_NAME}
sudo cp ${BASE_DIR}/${CRON_JOB_NAME} ${CRON_JOB_DEST}

echo "Enabling systemd services"
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}
sudo systemctl restart ${SERVICE_NAME}
