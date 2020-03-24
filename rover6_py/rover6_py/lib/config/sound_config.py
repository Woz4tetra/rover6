import os
from .config import Config


class SoundConfig(Config):
    def __init__(self):
        self.boot_sound = ""
        self.shutdown_sound = ""
        self.wifi_connect_sound = ""
        self.wifi_disconnect_sound = ""

        self.volume = 25

        super(SoundConfig, self).__init__("sounds.yaml")

        self.expand_user([
            self.boot_sound,
            self.shutdown_sound,
            self.wifi_connect_sound,
            self.wifi_disconnect_sound,
        ])

    def expand_user(self, args: list):
        for index, path in enumerate(args):
            args[index] = os.path.expanduser(path)

    def to_dict(self):
        return {
            "boot_sound"           : self.boot_sound,
            "shutdown_sound"       : self.shutdown_sound,
            "wifi_connect_sound"   : wifi_connect_sound,
            "wifi_disconnect_sound": wifi_disconnect_sound,
            "volume"               : self.volume,
        }
