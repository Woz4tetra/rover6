import os
import logging
from .config import Config


class WifiConfig(Config):
    def __init__(self):
        self.interface_name = "wlan0"
        super(WifiConfig, self).__init__("wifi.yaml")

    def to_dict(self):
        return {
            "interface_name": self.interface_name
        }
