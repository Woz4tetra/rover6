import time
import subprocess
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from .node import Node

wifi_config = ConfigManager.get_wifi_config()

logger = LoggerManager.get_logger()


class WifiHub(Node):
    UNKNOWN = 0
    WIFI = 1
    HOTSPOT = 2
    DISCONNECTED = 3

    def __init__(self, master):
        self.hostname = ""
        self.ip_default_str = "xx.xx.xx.xx"
        self.ip_address = self.ip_default_str
        self.interface_name = wifi_config.interface_name
        self.status_code = 0
        self.grep_ip_start_flag = "ip_address="
        self.prev_update_time = time.time()
        self.update_delay_s = wifi_config.update_delay_s

        self.sounds = None

        super(WifiHub, self).__init__(master)

    def start(self):
        self.sounds = self.master.sounds

    def update(self):
        if time.time() - self.prev_update_time < self.update_delay_s:
            return
        logger.debug("Updating wifi info")
        self.get_broadcast_status()
        self.get_hostname()
        self.prev_update_time = time.time()

    def run_cmd(self, cmd):
        proc = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
        stdout = proc.communicate()[0]
        return stdout.decode()

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
                status_code = self.WIFI
            else:
                status_code = self.DISCONNECTED
        # elif self.is_hotspot_running():
        #     self.status_code = self.HOTSPOT
        else:
            status_code = self.DISCONNECTED
        logger.debug("Broadcast status: %s" % status_code)

        if status_code != self.status_code:
            self.status_code = status_code
            if self.status_code == self.WIFI or self.status_code == self.HOTSPOT:
                self.sounds.wifi_connect()
            elif self.status_code == self.DISCONNECTED or self.status_code == self.UNKNOWN:
                self.sounds.wifi_disconnect()
