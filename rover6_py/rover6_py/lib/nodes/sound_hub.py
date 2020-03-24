import time
import subprocess
from omxplayer.player import OMXPlayer

from lib.nodes.node import Node
from lib.logger_manager import LoggerManager
from lib.config.config_manager import ConfigManager

logger = LoggerManager.get_logger()

sound_config = ConfigManager.get_sound_config()


class SoundController:
    def __init__(self, sound_paths):
        self.sound_paths = sound_paths
        self.players = self.get_players(sound_paths)

    def get_players(self, sound_paths: dict):
        players = {}
        for name, path in sound_paths.items():
            player = OMXPlayer(path, args=["-o", "alsa:hw:1,0"], dbus_name='org.mpris.MediaPlayer2.omxplayer1')
            player.pause()
            players[name] = player
        return players

    def play(self, track_name):
        player = self.players[track_name]
        player.set_position(0)
        player.play()

    def wait(self, track_name):
        while self.players[track_name].is_playing():
            time.sleep(0.1)

    def set_volume(self, percent):
        percent = max(0, min(percent, 100))
        subprocess.call("amixer -c 1 sset PCM {}%".format(percent), shell=True)

    def pause(self, track_name):
        self.players[track_name].pause()

    def quit_all(self):
        for name in self.sound_paths:
            self.players[name].quit()


class SoundHub(Node):
    def __init__(self, master):
        self.controller = SoundController(sound_config.sound_paths)
        super(SoundHub, self).__init__(master)

    def start(self):
        logger.info("Sound Hub initialized")
        self.controller.set_volume(sound_config.volume)
        logger.info("Playing boot sound")
        self.controller.play("boot_sound")
        self.controller.wait("boot_sound")

    def wifi_connect(self):
        logger.info("Playing wifi connect sound")
        self.controller.play("wifi_connect_sound")

    def wifi_disconnect(self):
        logger.info("Playing wifi disconnect sound")
        self.controller.play("wifi_disconnect_sound")

    def click(self):
        logger.info("Playing click sound")
        self.controller.play("click_sound")

    def stop(self):
        logger.info("Playing shutdown sound")
        self.controller.play("shutdown_sound")
        self.controller.wait("shutdown_sound")

    def __del__(self):
        self.controller.quit_all()
