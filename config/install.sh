#!/usr/bin/env bash

echo "Running rover6 config install"

BASE_DIR=$(realpath "$(dirname $0)")

hostnames_list=(rover6-hana
    rover6-dul
    rover6-set
    rover6-net
    rover6-daseot
    rover6-yeoseot
    rover6-ilgop
    rover6-yeodeol
    rover6-ahop
    rover6-yeol
)
welcome_messages=(하나 둘 셋 넷 다섯 여섯 일곱 여덟 아홉 열)
# welcome_messages=(1 2 3 4 5 6 7 8 9 10)
echo "Select a hostname:"
for i in ${!hostnames_list[@]}; do
    echo "$((i + 1)): ${hostnames_list[$i]}"
done

selected_hostname="2"

while [[ ${selected_hostname} == "" ]]; do
    printf "> "
    read selected_hostname
    if [ ${selected_hostname} -lt 1 -a ${selected_hostname} -gt 10 ]; then
        echo "Invalid selection: ${selected_hostname}"
        echo "Please select a number from 1-10"
        selected_hostname=""
    fi
done
welcome_character=${welcome_messages[$((selected_hostname - 1))]}
selected_hostname=${hostnames_list[$((selected_hostname - 1))]}
welcome_message="Welcome to ${selected_hostname}! ${welcome_character}"
# echo ${selected_hostname}
echo ${welcome_message}

echo ${selected_hostname} | sudo tee /etc/hostname > /dev/null
sed "s|{{HOSTNAME}}|$selected_hostname|g" ${BASE_DIR}/hosts | sudo tee /etc/hosts > /dev/null

if ! grep -qz "`cat ${BASE_DIR}/config_append.txt`" /boot/config.txt; then
    echo "Appending gpio config to /boot/config.txt"
    cat ${BASE_DIR}/config_append.txt | sudo tee -a /boot/config.txt > /dev/null
fi

if ! grep -qz "${welcome_message}" ~/.bashrc; then
    echo "Appending welcome message to ~/.bashrc"
    echo "echo \"${welcome_message}\"" | sudo tee -a ~/.bashrc > /dev/null
fi

bash ${BASE_DIR}/firmware/install.sh
bash ${BASE_DIR}/rover6_py/install.sh
bash ${BASE_DIR}/rover6_ros/installation/install.sh
