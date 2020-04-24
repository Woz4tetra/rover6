import re
import time
import logging
import subprocess
from logging import handlers

from lib.config import ConfigManager
from lib.logger_manager import MyFormatter
from .node import Node

data_log_config = ConfigManager.get_data_log_config()


class DataLogger(Node):
    start_flag = "---- Data Logger start -----"

    def __init__(self, master):
        self.logger = self._create_logger(**data_log_config.to_dict())

        self.cpu_temp_regex = r"temp=([\d.]*)'C"

        self.prev_log_time = time.time()
        self.log_timeout = 1.0 / data_log_config.log_freq_hz
        self.logger.info(self.start_flag)

        super(DataLogger, self).__init__(master)

    def update(self):
        if time.time() - self.prev_log_time < self.log_timeout:
            return
        self.prev_log_time = time.time()

        ina_data = self.master.rover.get("ina", "recv_time", "voltage_V", "current_mA")
        cpu_temp = self.get_cpu_temp()
        # camera_detected = self.get_camera_connected()

        self.log("ina", *ina_data)
        self.log("cpu_temp", cpu_temp)
        # self.log("camera_detected", camera_detected)

    def _call(self, *args):
        process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = process.communicate()
        return output.decode()

    def get_cpu_temp(self):
        output = self._call("vcgencmd", "measure_temp")
        match = re.search(self.cpu_temp_regex, output)
        if match:
            return match.group(1)
        else:
            return output

    def get_camera_connected(self):
        return self._call("vcgencmd", "get_camera").strip()

    def log(self, name, *data):
        data = list(map(str, data))
        data.insert(0, name)
        self.logger.info("\t".join(data))

    @staticmethod
    def _create_logger(**kwargs):
        name = kwargs["name"]
        path = kwargs["path"]
        level = kwargs["level"]
        format = kwargs["format"]
        suffix = kwargs["suffix"]

        logger = logging.getLogger(name)
        logger.setLevel(level)

        formatter = MyFormatter(format)

        rotate_handle = handlers.TimedRotatingFileHandler(
            path,
            when="D", interval=1
        )
        rotate_handle.setLevel(level)
        rotate_handle.setFormatter(formatter)
        rotate_handle.suffix = suffix
        logger.addHandler(rotate_handle)

        print_handle = logging.StreamHandler()
        print_handle.setLevel(level)
        print_handle.setFormatter(formatter)
        logger.addHandler(print_handle)

        return logger
