import time
import serial

from ..config import ConfigManager
from ..logger_manager import LoggerManager

device_port_config = ConfigManager.get_device_port_config()
logger = LoggerManager.get_logger()


class DevicePortWriteException(Exception):
    pass


class DevicePortReadException(Exception):
    pass


class DevicePort:
    def __init__(self):
        self.device = None

    def configure(self):
        logger.debug("Attempting to open address '%s'" % device_port_config.address)
        self.device = serial.Serial(
            device_port_config.address,
            device_port_config.baud_rate,
            timeout=device_port_config.timeout,
            write_timeout=device_port_config.write_timeout
        )

        # from https://www.raspberrypi.org/forums/viewtopic.php?t=61955
        self.out_buf_max_bytes = 16 * 12
        self.in_buf_max_bytes = 16 * 8

        self.read_buffer = b""
        self.write_buffer = b""

        # wait for the device to send data
        check_time = time.time()
        while self.in_waiting() < 0:
            time.sleep(0.001)

            if time.time() - check_time > device_port_config.write_timeout:
                logger.info(
                    "Waited for '%s' for %ss with no response..." % (
                        device_port_config.address, device_port_config.timeout)
                )
                return
        logger.info("%s is ready" % device_port_config.address)

    def write(self, packet: bytes):
        if not self.device:
            raise DevicePortWriteException("Device '%s' was never opened for writing" % device_port_config.address)
        if not self.is_open():
            raise DevicePortWriteException("Device '%s' is not open for writing" % device_port_config.address)
        self.device.write(packet)

    def out_waiting(self):
        """
        Safely check the serial buffer.
        :return: None if an OSError occurred, otherwise an integer value indicating the buffer size
        """
        try:
            return self.device.out_waiting
        except OSError:
            logger.error("Failed to check serial. Is there a loose connection?")
            raise    

    def in_waiting(self):
        """
        Safely check the serial buffer.
        :return: None if an OSError occurred, otherwise an integer value indicating the buffer size
        """
        try:
            return self.device.in_waiting
        except OSError:
            logger.error("Failed to check serial. Is there a loose connection?")
            raise

    def is_open(self):
        """Wrapper for isOpen"""
        return self.device.isOpen()
    
    def __read(self):
        if self.device.isOpen():
            self.read_buffer += self.device.read(self.in_waiting())
        else:
            raise DevicePortReadException("Serial port wasn't open for reading...")

    def read(self, size):
        while len(self.read_buffer) < size:
            self.__read()
            # time.sleep(0.001)
        result = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        return result

    def readline(self):
        newline_found = False

        while not newline_found:
            self.__read()
            try:
                index = self.read_buffer.index(b"\n")
                newline_found = True
            except ValueError:
                continue
        
        result = self.read_buffer[:index]
        self.read_buffer[index:]
        return result
    
    def stop(self):
        if self.is_open():
            logger.info("Closing device '{}'".format(device_port_config.address))
            self.device.close()

    # def __del__(self):
    #     self.stop()
