import os
from .config import Config


class SoundConfig(Config):
    def __init__(self):
        self.boot_sound = ""
        self.shutdown_sound = ""
        super(SoundConfig, self).__init__("sounds.yaml")
        self.boot_sound = os.path.expanduser(self.boot_sound)
        self.shutdown_sound = os.path.expanduser(self.shutdown_sound)
