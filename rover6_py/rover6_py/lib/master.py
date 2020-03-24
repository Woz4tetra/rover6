from lib.nodes.rover import RoverClient
from lib.nodes.gpio_hub import GpioHub
from lib.nodes.sound_hub import SoundHub
from lib.nodes.wifi_hub import WifiHub
from lib.nodes.data_logger import DataLogger


class Master:
    def __init__(self):
        self.rover = RoverClient(self)
        self.gpio_hub = GpioHub(self)
        self.sounds = SoundHub(self)
        self.wifi = WifiHub(self)
        self.data_logger = DataLogger(self)

    def start(self):
        self.sounds.start()
        self.gpio_hub.start()

        self.wifi.start()
        self.data_logger.start()

        self.rover.start()

    def update(self):
        self.gpio_hub.update()
        self.wifi.update()
        self.rover.update()
        self.sounds.update()
        self.data_logger.update()

    def stop(self):
        self.sounds.stop()
        self.rover.stop()
        self.gpio_hub.stop()
        self.wifi.stop()
        self.data_logger.stop()
