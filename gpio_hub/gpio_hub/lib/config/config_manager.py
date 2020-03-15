from .log_config import LogConfig
from .gpio_config import GpioConfig
from .sound_config import SoundConfig


class ConfigManager:
    log_config = None
    gpio_config = None
    sound_config = None

    def __init__(self):
        raise Exception("{} is class only".format(self.__class__.__name__))

    @classmethod
    def get_log_config(cls):
        if cls.log_config is None:
            cls.log_config = LogConfig()
        return cls.log_config

    @classmethod
    def get_gpio_config(cls):
        if cls.gpio_config is None:
            cls.gpio_config = GpioConfig()
        return cls.gpio_config

    @classmethod
    def get_sound_config(cls):
        if cls.sound_config is None:
            cls.sound_config = SoundConfig()
        return cls.sound_config