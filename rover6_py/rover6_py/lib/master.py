
from lib.rover.rover_client import RoverClient
from lib.gpio_hub import GpioHub
from lib.sound_hub import SoundHub
from lib.wifi_hub import WifiHub
from lib.data_logger import DataLogger

logger = LoggerManager.get_logger()

class Master:
    def __init__(self):
        self.rover = RoverClient(self)
        self.gpio_hub = GpioHub(self)
        self.sounds = SoundHub(self)
        self.wifi = WifiHub(self)
        self.data_logger = DataLogger(roverself)

        self.nodes = [
            self.rover,
            self.gpio_hub,
            self.sounds,
            self.wifi,
            self.data_logger,
        ]

    def start(self):
        for node in self.nodes:
            node.start()

    def update(self):
        for node in self.nodes:
            node.update()

    def stop(self):
        for node in self.nodes:
            node.stop()
