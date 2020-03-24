import sys
import time
import numpy as np
import subprocess

# from lib.joystick import Joystick
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException

logger = LoggerManager.get_logger()

sound_config = ConfigManager.get_sound_config()


def shutdown():
    logger.warn("Shutdown function called. Shutting down everything.")
    sounds.play(sound_config.shutdown_sound)
    rover.stop()
    logger.info(rover.data_frame)
    # gpio_hub.close()
    while sounds.is_running():
        time.sleep(0.1)
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()


def close():
    logger.info("Close function called. Exiting\n\n")
    sounds.play(sound_config.shutdown_sound)
    while sounds.is_running():
        time.sleep(0.1)

    rover.stop()
    logger.info(rover.data_frame)
    for identifier, times in rover.recv_times.items():
        logger.info("%s:\t%0.4fHz" % (identifier, 1 / np.mean(np.diff(times))))
        # print("%s:\t%s" % (identifier, np.diff(times).tolist()))
    logger.info("Closing GPIO Hub\n\n")
    gpio_hub.close()
    sounds.quit()


def main():
    logger.info("Starting rover\n\n")

    rover.set_wifi_hub(wifi)
    rover.set_gpio_hub(gpio_hub)

    gpio_hub.set_fan(100)

    sounds.set_volume(sound_config.volume)
    time.sleep(0.15)
    sounds.play(sound_config.boot_sound)

    gpio_hub.update()

    try:
        rover.start()

        while True:
            gpio_hub.update()
            wifi.update()
            data_logger.update()
            time.sleep(1.0 / gpio_config.update_rate_hz)

            if not rover.read_task_running():
                logger.error("Error detected in read task. Raising exception")
                raise rover.thread_exception
    except LowBatteryException:
        shutdown()
    except ShutdownException:
        shutdown()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
        close()


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
