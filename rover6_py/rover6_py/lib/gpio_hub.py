import time
import math
from RPi import GPIO
from .config.config_manager import ConfigManager
from .logger_manager import LoggerManager

logger = LoggerManager.get_logger()
gpio_config = ConfigManager.get_gpio_config()

from .exceptions import ShutdownException


class GpioHub:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio_config.led_out, GPIO.OUT)
        GPIO.setup(gpio_config.fan_out, GPIO.OUT)
        GPIO.setup(gpio_config.button_in, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin set as input w/ pull-up

        self.led_pwm = GPIO.PWM(gpio_config.led_out, 50)  # Initialize PWM with 100Hz frequency
        self.led_duty_cycle = 0

        self.fan_pwm = GPIO.PWM(gpio_config.fan_out, 50)
        self.fan_duty_cycle = 0

        self.led_pwm.start(self.led_duty_cycle)
        self.fan_pwm.start(self.fan_duty_cycle)

        self.shutdown_timer = time.time()

        self.led_cycle_direction = False
        self.prev_button_state = False
        self.button_state = False
        self.time_print_out = None

        logger.info("GPIO Hub initialized")

    def is_button_pressed(self):
        return not GPIO.input(gpio_config.button_in)

    def set_led(self, duty_cycle):
        self.led_duty_cycle = duty_cycle
        self.led_pwm.ChangeDutyCycle(self.led_duty_cycle)

    def set_fan(self, duty_cycle):
        self.fan_duty_cycle = duty_cycle
        self.fan_pwm.ChangeDutyCycle(self.fan_duty_cycle)

    def log_shutdown_timer(self, time_offset):
        time_print_out = math.ceil(gpio_config.shutdown_time_s - time_offset)
        if time_print_out != self.time_print_out:
            logger.info("\t%s..." % time_print_out)
            self.time_print_out = time_print_out

    def update(self):
        self.button_state = self.is_button_pressed()
        if self.button_state != self.prev_button_state:
            if self.button_state:
                logger.info("Shutdown button is being held. Shutting down in...")
                self.shutdown_timer = time.time()
            else:
                self.time_print_out = None
                logger.info("Shutdown button released. Shutdown cancelled")

            self.prev_button_state = self.button_state
        if self.button_state:
            if self.led_cycle_direction:
                cycle = self.led_duty_cycle + gpio_config.led_pwm_rate
                if cycle > 100:
                    cycle = 100
                    self.led_cycle_direction = False
            else:
                cycle = self.led_duty_cycle - gpio_config.led_pwm_rate
                if cycle < 0:
                    cycle = 0
                    self.led_cycle_direction = True
            self.set_led(cycle)

            time_offset = time.time() - self.shutdown_timer
            self.log_shutdown_timer(time_offset)

            if time_offset > gpio_config.shutdown_time_s:
                logger.info("Starting shutdown routine")
                raise ShutdownException
        else:
            self.set_led(100)

    def close(self):
        # self.led_pwm.stop()
        self.fan_pwm.stop()
        GPIO.cleanup()
