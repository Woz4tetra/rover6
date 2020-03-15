import os
import logging
from .config import Config


class LogConfig(Config):
    def __init__(self):
        self.name = "gpio_hub"
        self.level = logging.DEBUG
        self.file_name = "gpio_hub"
        self.suffix = "%Y-%m-%d.log"
        self.format = "%(levelname)s\t%(asctime)s\t[%(name)s, %(filename)s:%(lineno)d]\t%(message)s"

        super(LogConfig, self).__init__("logging.yaml")

        self.dir = os.path.join(self.base_dir, "logs")
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        self.path = os.path.join(self.dir, self.file_name)