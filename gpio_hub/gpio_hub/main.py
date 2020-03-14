import sys
import time
import subprocess

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.gpio_hub import GpioHub, ShutdownException

logger = LoggerManager.get_logger()
gpio_hub = GpioHub()
gpio_config = ConfigManager.get_gpio_config()

def shutdown():
    logger.warning("Shutdown function called. Shutting down everything.")
    # gpio_hub.close()
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()

def main():
    logger.info("Starting GPIO Hub\n\n")
    gpio_hub.set_fan(100)

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
