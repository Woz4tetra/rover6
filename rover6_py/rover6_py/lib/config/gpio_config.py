import os
import logging
from .config import Config


class GpioConfig(Config):
    def __init__(self):
        self.led_out = 18
        self.button_in = 27
        # self.unlatch_out = 17  # unlatch handled by /boot/config.txt
        self.fan_out = 22
        self.shutdown_time_s = 3.0
        super(GpioConfig, self).__init__("gpio.yaml")
