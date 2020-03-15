from .config import Config


class DevicePortConfig(Config):
    def __init__(self):
        self.baud_rate = 500000
        self.address = "/dev/serial0"
        self.timeout = None
        self.write_timeout = None
        self.update_rate_hz = 15
        super(DevicePortConfig, self).__init__("device_port.yaml")
