import sys
import time
import subprocess

# from lib.joystick import Joystick
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException
from lib.master import Master

logger = LoggerManager.get_logger()

general_config = ConfigManager.get_general_config()


def shutdown(master):
    logger.warn("Shutdown function called. Shutting down everything.")
    master.stop()
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()


def close(master):
    logger.info("Close function called. Exiting\n\n")
    master.stop()


def main():
    logger.info("Starting rover")
    master = Master()
    update_delay = 1.0 / general_config.update_rate_hz
    try:
        master.start()
        while True:
            master.update()
            time.sleep(update_delay)
    except LowBatteryException as e:
        logger.error(str(e), exc_info=True)
        shutdown(master)
    except ShutdownException as e:
        logger.error(str(e), exc_info=True)
        shutdown(master)
    except BaseException as e:
        logger.error(str(e), exc_info=True)
        close(master)


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
