import os
import logging
from .config import Config


class GpioConfig(Config):
    def __init__(self):
        self.led_out = 18
        self.button_in = 27
        # self.unlatch_out = 17  # unlatch handled by /boot/config.txt
        self.fan_out = 22
        self.lidar_out = 23
        self.shutdown_time_s = 3.0
        super(GpioConfig, self).__init__("gpio.yaml")

    def to_dict(self):
        return {
            "led_out": self.led_out,
            "button_in": self.button_in,
            "fan_out": self.fan_out,
            "shutdown_time_s": self.shutdown_time_s,
        }
