import os
import time
import signal
import subprocess

from lib.nodes.node import Node

from lib.logger_manager import LoggerManager
from lib.config.config_manager import ConfigManager

logger = LoggerManager.get_logger()

sound_config = ConfigManager.get_sound_config()


class SoundController:
    def __init__(self):
        self.sound_played = False
        self.process = None

    def play(self, uri):
        self.sound_played = True
        if type(uri) != str or len(uri) == 0:
            return

        if not self.is_running():
            self.process = subprocess.Popen(["omxplayer", "-o", "alsa:hw:1,0", uri], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, #close_fds=True
            )

    def wait(self, timeout=None):
        try:
            stdout, stderr = self.process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.stop_track()
            stdout, stderr = proc.communicate()
        if len(stdout) > 0:
            logger.info(stdout.decode())
        if len(stderr) > 0:
            logger.error(stderr.decode())
        self.sound_played = False
        # while self.is_running():
        #     time.sleep(0.1)

    def get_output(self):
        stdout = self.process.stdout.read()
        stderr = self.process.stderr.read()
        return stdout, stderr

    def send(self, command):
        # if self.process is not None:
        if self.is_running():
            self.process.stdin.write(command)

    def set_volume(self, percent):
        percent = max(0, min(percent, 100))
        subprocess.call("amixer -c 1 sset PCM {}%".format(percent), shell=True)

    def quit(self):
        self.send(b'q')
        self.process = None

    def interrupt(self):
        os.kill(self.process.pid, signal.SIGINT)
        self.process = None

    def kill(self):
        os.kill(self.process.pid, signal.SIGKILL)
        self.process = None

    def toggle_pause(self):
        self.send(b'p')

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def stop_track(self):
        self.quit()
        time.sleep(0.25)
        if self.is_running():  # still active:
            self.interrupt()
            time.sleep(1.0)
        if self.is_running():  # still active:
            self.kill()
        if self.is_running():
            raise RuntimeError("Failed to stop sound process %s" % self.process.pid)


class SoundHub(Node):
    def __init__(self, master):
        self.controller = SoundController()
        super(SoundHub, self).__init__(master)

    def start(self):
        logger.info("Sound Hub initialized")
        self.controller.set_volume(sound_config.volume)
        logger.info("Playing boot sound")
        self.controller.play(sound_config.boot_sound)
        self.controller.wait()

    def update(self):
        if not self.controller.is_running() and self.controller.sound_played:
            stdout, stderr = self.controller.get_output()
            if len(stdout) > 0:
                logger.info(stdout.decode())
            if len(stderr) > 0:
                logger.error(stderr.decode())

    def wifi_connect(self):
        logger.info("Playing wifi connect sound")
        self.controller.play(sound_config.wifi_connect_sound)

    def wifi_disconnect(self):
        logger.info("Playing wifi disconnect sound")
        self.controller.play(sound_config.wifi_disconnect_sound)

    def click(self):
        logger.info("Playing click sound")
        self.controller.play(sound_config.click_sound)

    def stop(self):
        logger.info("Playing shutdown sound")
        self.controller.play(sound_config.shutdown_sound)
        self.controller.wait()

    def __del__(self):
        self.controller.stop_track()


if __name__ == '__main__':
    def main():
        sounds = SoundController()
        sounds.play("/home/pi/Music/Pokémon Center (Day).mp3")
        time.sleep(3)
        sounds.quit()
        while True:
            time.sleep(0.5)
            print(sounds.is_running())


    main()
