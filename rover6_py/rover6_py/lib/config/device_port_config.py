from .config import Config


class DevicePortConfig(Config):
    def __init__(self):
        self.baud_rate = 115200
        self.address = "/dev/serial0"
        self.timeout = 5.0
        self.write_timeout = 5.0
        self.update_rate_hz = 30
        super(DevicePortConfig, self).__init__("device_port.yaml")
