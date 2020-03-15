import os
import time
import signal
import subprocess


class SoundHub:
    process = None

    # def __init__(self):
    #     self.process = None

    def play(self, uri):
        if type(uri) != str or len(uri) == 0:
            return
        if not self.is_running():
            self.__class__.process = subprocess.Popen(["omxplayer", "-o", "alsa:hw:1,0", uri], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=None, bufsize=0, close_fds=True)

    def send(self, command):
        if self.process is not None:
            self.process.stdin.write(command)

    def set_volume(self, percent):
        percent = max(0, min(percent, 100))
        subprocess.call("amixer -c 1 sset PCM {}%".format(percent), shell=True)

    def quit(self):
        self.send(b'q')
        self.process = None

    def stop(self):
        os.kill(self.process.pid, signal.SIGINT)
        self.process = None

    def kill(self):
        os.kill(self.process.pid, signal.SIGKILL)
        self.process = None

    def toggle_pause(self):
        self.send(b'p')

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def __del__(self):
        self.quit()
        time.sleep(0.25)
        if self.is_running():  # still active:
            self.stop()
            time.sleep(1.0)
        if self.is_running():  # still active:
            self.kill()
        if self.is_running():
            raise RuntimeError("Failed to stop SoundHub process %s" % self.process.pid)


if __name__ == '__main__':
    def main():
        sound_hub = SoundHub()
        sound_hub.play("/home/pi/Music/Pokémon Center (Day).mp3")
        time.sleep(3)
        sound_hub.quit()
        while True:
            time.sleep(0.5)
            print(sound_hub.is_running())
    main()
