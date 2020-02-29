import os
import subprocess

class AutohotspotManager:
    UNKNOWN = 0
    WIFI = 1
    HOTSPOT = 2
    DISCONNECTED = 3

    def __init__(self):
        self.status_code = 0
        self.hostname = ""
        self.ip_default_str = "xx.xx.xx.xx"
        self.ip_address = self.ip_default_str
        self.interface_name = "wlan0"

        self.grep_ip_start_flag = "ip_address="
        self.config_path = "/home/pi/.local/rover6/config/autohotspot.conf"

    def run_cmd(self, cmd):
        proc = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
        stdout = proc.communicate()[0]
        return stdout.decode()

    def is_hotspot_running(self):
        cmd = "systemctl status hostapd | grep \"(running)\""
        return len(self.run_cmd(cmd)) > 0

    def is_wifi_running(self):
        cmd = "wpa_cli status | grep \"%s\"" % self.interface_name
        return self.interface_name in self.run_cmd(cmd)

    def get_wifi_ip(self):
        cmd = "wpa_cli -i \"%s\" status | grep 'ip_address'" % self.interface_name
        output = self.run_cmd(cmd).strip()
        if output.startswith(self.grep_ip_start_flag):
            self.ip_address = output[len(self.grep_ip_start_flag):]
        else:
            self.ip_address = self.ip_default_str

    def get_hostname(self):
        self.hostname = self.run_cmd("hostname").strip()

    def get_broadcast_status(self):
        if self.is_wifi_running():
            self.get_wifi_ip()
            if self.ip_address:
                self.status_code = self.WIFI
            else:
                self.status_code = self.DISCONNECTED
        elif self.is_hotspot_running():
            self.status_code = self.HOTSPOT
        else:
            self.status_code = self.DISCONNECTED

    def update(self):
        self.get_broadcast_status()
        self.get_hostname()

    def set_hotspot_config(self, force_hotspot):
        with open(self.config_path, 'w') as file:
            file.write("#!/bin/bash\nFORCE_HOTSPOT=%s\n" % str(bool(force_hotspot)).lower())

    def set_mode(self, mode=WIFI):
        # cron is running autohotspot every minute. The updated config will
        # go into effect after a minute
        if mode == self.WIFI:
            self.set_hotspot_config(False)
            # cmd = "sudo /usr/bin/autohotspot"
        elif mode == self.HOTSPOT:
            self.set_hotspot_config(True)
            # cmd = "sudo /usr/bin/autohotspot true"
        else:
            return False
        # self.run_cmd(cmd)
        return True
