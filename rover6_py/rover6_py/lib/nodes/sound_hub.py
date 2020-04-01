import os
import time
import dbus
import subprocess
from omxplayer.player import OMXPlayer

from lib.nodes.node import Node
from lib.logger_manager import LoggerManager
from lib.config.config_manager import ConfigManager

logger = LoggerManager.get_logger()

sound_config = ConfigManager.get_sound_config()


class SoundController:
    def __init__(self):
        self.players = {}

    def get_player(self, path: str):
        return OMXPlayer(path, args=["-o", "alsa:hw:1,0"], dbus_name='org.mpris.MediaPlayer2.omxplayer1')

    def play(self, path):
        track_name = os.path.basename(path)
        try:
            is_playing = track_name in self.players and self.players[track_name].is_playing()
            if is_playing:
                self.players[track_name].set_position(0)
        except BaseException as e:
            is_playing = False
            logger.debug("No longer playing. Reason: %s" % str(e))

        if not is_playing:
            self.players[track_name] = self.get_player(path)

        return track_name

    def wait(self, track_name):
        try:
            while self.players[track_name].is_playing():
                time.sleep(0.1)
        except BaseException as e:
            logger.debug("%s player is already stopped. Reason: %s" % (track_name, e))

    def set_volume(self, percent):
        percent = max(0, min(percent, 100))
        subprocess.call("amixer -c 1 sset PCM {}%".format(percent), shell=True)

    def pause(self, track_name):
        self.players[track_name].pause()

    def quit_all(self):
        for player in self.players.values():
            player.quit()


class SoundHub(Node):
    def __init__(self, master):
        self.controller = SoundController()
        super(SoundHub, self).__init__(master)

    def start(self):
        logger.info("Sound Hub initialized")
        self.controller.set_volume(sound_config.volume)
        logger.info("Playing boot sound")
        self.wait(self.play(sound_config.boot_sound))

    def wifi_connect(self):
        logger.info("Playing wifi connect sound")
        self.play(sound_config.wifi_connect_sound)

    def wifi_disconnect(self):
        logger.info("Playing wifi disconnect sound")
        self.play(sound_config.wifi_disconnect_sound)

    def click(self):
        logger.info("Playing click sound")
        self.play(sound_config.click_sound)

    def stop(self):
        logger.info("Playing shutdown sound")
        self.wait(self.play(sound_config.shutdown_sound))

    def play(self, path):
        return self.controller.play(path)

    def wait(self, track_name):
        self.controller.wait(track_name)

    def __del__(self):
        self.controller.quit_all()
