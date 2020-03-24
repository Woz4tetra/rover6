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

        self.sound_paths = {
            "boot_sound": self.boot_sound,
            "shutdown_sound": self.shutdown_sound,
            "wifi_connect_sound": self.wifi_connect_sound,
            "wifi_disconnect_sound": self.wifi_disconnect_sound,
        }

        self.check_path(self.sound_paths)

    def check_path(self, kwargs: dict):
        for name, path in kwargs.items():
            abs_path = os.path.expanduser(path)
            if not os.path.isfile(abs_path):
                raise ValueError("Can't find sound file: '%s'")
            kwargs[name] = abs_path
            self.__dict__[name] = abs_path


    def to_dict(self):
        return {
            "boot_sound"           : self.boot_sound,
            "shutdown_sound"       : self.shutdown_sound,
            "wifi_connect_sound"   : wifi_connect_sound,
            "wifi_disconnect_sound": wifi_disconnect_sound,
            "volume"               : self.volume,
        }
