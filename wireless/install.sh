#!/usr/bin/env bash

echo "Running hotspot install script"

BASE_DIR=$(realpath "$(dirname $0)")

# install dependencies
sudo apt-get update
#sudo apt-get upgrade
sudo apt-get install hostapd dnsmasq   --assume-yes

sudo systemctl stop dnsmasq
sudo systemctl stop hostapd

# dhcpcd config installation
cat ${BASE_DIR}/dhcpcd_append.txt | sudo tee -a /etc/dhcpcd.conf > /dev/null

# dnsmasq config installation
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
cp ${BASE_DIR}/dnsmasq.conf /etc/dnsmasq.conf

sudo systemctl enable dnsmasq

# hostapd config installation
rover_hostname=`hostname`
sed "s|{{HOSTNAME}}|$rover_hostname|g" ${BASE_DIR}/hostapd.conf | sudo tee /etc/hostapd/hostapd.conf > /dev/null

echo "DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"" | sudo tee -a /etc/default/hostapd > /dev/null

# start services
sudo service dhcpcd restart
sudo systemctl start dnsmasq

sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd

sudo systemctl status dhcpcd
sudo systemctl status hostapd
sudo systemctl status dnsmasq

# setup iptables
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf > /dev/null

# route internet traffic from wlan1 to wlan0
sudo iptables -t nat -A  POSTROUTING -o wlan0 -j MASQUERADE

# save iptables rules
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"

# load rules at boot
sed '$iiptables-restore < /etc/iptables.ipv4.nat' /etc/rc.local | sudo tee /etc/rc.local > /dev/null

echo "Hotspot installation complete!"
