import sys
import time
import subprocess

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.gpio_hub import GpioHub, ShutdownException
from lib.sound_hub import SoundHub

logger = LoggerManager.get_logger()
gpio_hub = GpioHub()
gpio_config = ConfigManager.get_gpio_config()
sound_config = ConfigManager.get_sound_config()

sounds = SoundHub()

def shutdown():
    sounds.play(sound_config.shutdown_sound)
    time.sleep(0.5)
    logger.warning("Shutdown function called. Shutting down everything.")
    # gpio_hub.close()
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()

def main():
    logger.info("Starting GPIO Hub\n\n")
    gpio_hub.set_fan(100)

    sounds.set_volume(25)
    sounds.play(sound_config.boot_sound)

    while True:
        gpio_hub.update()
        time.sleep(1.0 / gpio_config.update_rate_Hz)


if __name__ == "__main__":
    try:
        main()
    except ShutdownException as e:
        shutdown()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
    finally:
        logger.info("Closing GPIO Hub\n\n")
        gpio_hub.close()
