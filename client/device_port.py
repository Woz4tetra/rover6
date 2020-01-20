import time
import serial
import logging

from default_params import *


class DevicePortWriteException(Exception):
    pass

class DevicePortReadException(Exception):
    pass


class DevicePort:
    def __init__(self, address, log_level=logging.DEBUG, device=None, start_time=None, first_packet="", whoiam="", baud=DEFAULT_RATE):
        """
        Wraps the serial.Serial class and implements the atlasbuggy serial protocol for arduinos

        :param address: USB serial address string
        :param log_level: log level for debugging
        :param device: an instance of serial.Serial
        :param start_time: device unix timestamp start time
        :param first_packet: initialization data sent by the arduino at the start
        :param whoiam: whoiam ID indicating which Arduino class should be matched to which serial port
        """
        self.address = address

        self.device = device
        self.start_time = start_time
        self.baud = baud

        self.buffer = ''  # current packet buffer

        self.make_logger(log_level)

    def make_logger(self, level):
        print_handle = logging.StreamHandler()
        print_handle.setLevel(level)
        self.logger = logging.getLogger("Device Factory")

        formatter = logging.Formatter(DEFAULT_LOG_FORMAT)
        print_handle.setFormatter(formatter)
        self.logger.addHandler(print_handle)

    def configure(self):
        self.logger.debug("Attempting to open address '%s'" % self.address)
        self.device = serial.Serial(self.address, self.baud)

        # wait for the device to send data
        check_time = time.time()
        while self.in_waiting() < 0:
            time.sleep(0.001)

            if time.time() - check_time > PROTOCOL_TIMEOUT:
                self.logger.info(
                    "Waited for '%s' for %ss with no response..." % (self.address, PROTOCOL_TIMEOUT)
                )
                return
        self.logger.debug("%s is ready" % self.address)
        time.sleep(0.5)  # wait for the device to boot

    @classmethod
    def reinit(cls, kwargs):
        """Reinitialize a device port. All ports are configured at this point. Use supplied constructor values."""
        return DevicePort(**kwargs)

    def check_protocol(self, ask_packet, recv_packet_header, protocol_timeout=None):
        """
        A call and response method. After an "ask packet" is sent, the process waits for
        a packet with the expected header for 2 seconds

        For initialization

        :param ask_packet: packet to send
        :param recv_packet_header: what the received packet should start with
        :return: the packet received with the header and packet end removed
        """

        if protocol_timeout is None:
            protocol_timeout = PROTOCOL_TIMEOUT

        if ask_packet:
            self.logger.debug("Checking '%s' protocol at '%s'" % (ask_packet, self.address))
            self.write(ask_packet)
        else:
            self.logger.debug("Checking '%s' protocol at '%s'" % (recv_packet_header, self.address))

        protocol_start_time = time.time()
        abides_protocol = False
        answer_packet = None
        attempts = 0
        rounded_time = 0

        # wait for the correct response
        while not abides_protocol:
            in_waiting = self.in_waiting()
            if in_waiting > 0:
                if self.start_time is None:
                    self.start_time = time.time()

                packet_time, packet = self.readline()
                self.logger.debug("Got packet: %s from '%s'" % (repr(packet), self.address))
                if packet is None:
                    return None

                # return None if read failed
                if packet is None:
                    raise DevicePortReadException(
                        "Serial read failed for address '%s'... Board never signalled ready" % self.address)

                # parse received packet
                if len(packet) == 0:
                    self.logger.debug("Empty packet from '%s'! Contained only \\n" % self.address)
                    continue
                if packet[0:len(recv_packet_header)] == recv_packet_header:  # if the packet starts with the header,
                    self.logger.debug("received packet: %s from '%s'" % (repr(packet), self.address))

                    answer_packet = packet[len(recv_packet_header):]  # record it and return it

                    abides_protocol = True

            prev_rounded_time = rounded_time
            rounded_time = int((time.time() - protocol_start_time) * 5)
            if rounded_time > protocol_timeout and rounded_time % 3 == 0 and prev_rounded_time != rounded_time:
                attempts += 1
                if ask_packet:
                    self.logger.debug("Writing '%s' again to '%s'" % (ask_packet, self.address))
                else:
                    self.logger.debug("Still waiting for '%s' packet from '%s'" % (recv_packet_header, self.address))

                if ask_packet:
                    self.write(ask_packet)

            # return None if operation timed out
            if (time.time() - protocol_start_time) > protocol_timeout:
                if ask_packet:
                    raise DevicePortReadException(
                        "Didn't receive response for packet '%s' on address '%s'. "
                        "Operation timed out after %ss." % (
                            ask_packet, self.address, protocol_timeout))
                else:
                    raise DevicePortReadException(
                        "Didn't receive response waiting for packet '%s' on address '%s'. "
                        "Operation timed out after %ss" % (
                            recv_packet_header, self.address, protocol_timeout))

        return answer_packet  # when the while loop exits, abides_protocol must be True

    def write(self, packet):
        """Write a string. "packet" should not have a new line in it. This is sent for you."""
        data = bytearray(str(packet) + PACKET_END, 'ascii')
        if not self.device:
            raise DevicePortWriteException("Device '%s' was never opened for writing" % self.address)
        if not self.is_open():
            raise DevicePortWriteException("Device '%s' is not open for writing" % self.address)
        self.device.write(data)

    def in_waiting(self):
        """
        Safely check the serial buffer.
        :return: None if an OSError occurred, otherwise an integer value indicating the buffer size
        """
        try:
            return self.device.inWaiting()
        except OSError:
            self.logger.error("Failed to check serial. Is there a loose connection?")
            raise

    def is_open(self):
        """Wrap isOpen for the Arduino class"""
        return self.device.isOpen()

    def readline(self):
        """
        Read until the next new line character

        For initialization use.
        """
        packet_time = time.time()
        if self.device.isOpen():
            incoming = self.device.readline()
        else:
            raise RuntimeError("Serial port wasn't open for reading...")

        if len(incoming) > 0:
            # append to the buffer
            try:
                packet = incoming.decode("utf-8", "ignore")
                return packet_time, packet[:-1]  # remove \n
            except UnicodeDecodeError:
                self.logger.debug("Found non-ascii characters! '%s'" % incoming)
                raise
        else:
            return packet_time, None

    def read(self, in_waiting):
        """
        Read all available data on serial and split them into packets as
        indicated by packet_end.

        For initialization and process use
        """
        # read every available character
        receive_time = time.time()
        if self.device.isOpen():
            incoming = self.device.read(in_waiting)
        else:
            raise RuntimeError("Serial port wasn't open for reading...")

        if len(incoming) > 0:
            # append to the buffer
            try:
                self.buffer += incoming.decode("utf-8", "ignore")
            except UnicodeDecodeError:
                self.logger.debug("Found non-ascii characters! '%s'" % incoming)
                raise

            if len(self.buffer) > len(PACKET_END):
                # split based on user defined packet end
                packets = self.buffer.split(PACKET_END)

                # reset the buffer. If the buffer ends with \n, the last element in packets will be an empty string
                self.buffer = packets.pop(-1)

                return receive_time, packets
        return receive_time, []

    def stop(self):
        self.device.close()
