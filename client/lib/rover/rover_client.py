import time
import struct
import datetime
import threading

from . import tof_obstacles
from ..device_port import DevicePort, DevicePortReadException, DevicePortWriteException
from ..config import ConfigManager
from ..logger_manager import LoggerManager

rover_config = ConfigManager.get_rover_config()
device_port_config = ConfigManager.get_device_port_config()
logger = LoggerManager.get_logger()


def get_checksum(b: bytes):
    checksum = 0
    for val in b:
        checksum += val
    checksum &= 0xffff
    return checksum


class PacketBuffer:
    WRITE_CODES = {}

    def __init__(self, packet: bytes):
        self.packet = packet
        self.packet_str = packet.decode()
        self.index = 0
        self.recv_checksum = 0
        self.calc_checksum = 0

        self.type_mapping = {
            'd': int,
            'u': int,
            'f': float,
            's': str,
        }

        self.packet_num = int(self.next())
        self.identifier = int(self.next())
        self.code = self.WRITE_CODES[self.identifier]

    def next(self) -> bytes:
        index = self.packet_str.find("\t")
        if index < 0:
            raise StopIteration

        segment = self.packet[self.index: index]
        self.index = index
        return segment

    def checksum(self) -> bool:
        self.recv_checksum = int.from_bytes(self.packet[-2:], "big")
        self.calc_checksum = get_checksum(self.packet[:-2])
        return self.recv_checksum == self.calc_checksum

    def iter(self):
        segment_index = 0
        while self.index < len(self.packet) - 2:  # ignore checksum bytes
            segment = self.next()
            segment_type = self.code[segment_index]
            yield segment_index, self.type_mapping[segment_type](segment)
            segment_index += 1


class RoverClient:
    PACKET_CODES = {
        "txrx" : "dd",
        "bno"  : "ufffffffffd",
        "enc"  : "uddff",
        "fsr"  : "udd",
        "safe" : "udddddddddddd",
        "ina"  : "ufff",
        "ir"   : "udd",
        "servo": "udddddddddddddddd",
        "tof"  : "udddddd",
        "ready": "us",
    }

    NUM_SERVOS = 16

    PACKET_NAMES = {
        "ina"   : ["time", "current_mA", "power_mW", "voltage_V"],
        "enc"   : ["time", "posA", "posB", "speedA", "speedB"],
        "fsr"   : ["time", "left", "right"],
        "ir"    : ["time", "type", "value"],
        "bno"   : ["time", "orientation_x", "orientation_y", "orientation_z", "gyro_x", "gyro_y", "gyro_z", "accel_x",
                   "accel_y", "accel_z", "temperature"],
        "tof"   : ["time", "front_mm", "back_mm", "front_measure_status", "back_measure_status", "front_status",
                   "back_status"],
        "servo" : ["time"] + [str(i) for i in range(NUM_SERVOS)],
        "safety": ["time", "is_left_bumper_trig", "is_right_bumper_trig", "is_front_tof_trig", "is_back_tof_trig",
                   "is_front_tof_ok", "is_back_tof_ok", "are_servos_active", "are_motors_active", "voltage_ok",
                   "is_active", "is_reporting_enabled", "is_speed_pid_enabled"],
        "txrx"  : ["packet_num", "success"],
        "ready" : ["time", "name"]
    }

    WRITE_COMMANDS = {
        "toggle_active"        : "<>",
        "get_ready"            : "?",
        "toggle_reporting"     : "[]",
        "update_time_str"      : "t",
        "set_motors"           : "m",
        "set_pid_ks"           : "ks",
        "set_servo"            : "s",
        "set_servo_default"    : "sd",
        "set_safety_thresholds": "safe"
    }

    WRITE_CODES = {
        "toggle_active"        : "d",
        "get_ready"            : "d",
        "toggle_reporting"     : "d",
        "update_time_str"      : "s",
        "set_motors"           : "ff",
        "set_pid_ks"           : "ffffff",
        "set_servo"            : "dd",
        "set_servo_default"    : "d",
        "set_safety_thresholds": "dddd"
    }
    PacketBuffer.WRITE_CODES = WRITE_CODES

    CODE_TO_TYPE = {
        'd': int,
        'l': int,
        'f': float,
        's': str,
    }

    def __init__(self):
        self.device = DevicePort()
        self.data_frame = {header: None for header in self.PACKET_CODES.keys()}

        self.name_index_mapping = {}
        for identifier, names in self.PACKET_NAMES.items():
            self.name_index_mapping[identifier] = {name: index for index, name in enumerate(names)}

        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.thread = threading.Thread(target=self.read_task, args=(lambda: self.should_stop,))

        self.recv_times = {}
        self.read_update_rate_hz = 120.0
        self.prev_packet_time = 0.0

        self.write_lock = threading.Lock()

        self.packet_start = b"\x12\x34"
        self.packet_stop = b"\n"

        self.read_packet_num = 0
        self.write_packet_num = 0

    def start(self):
        self.device.configure()
        self.check_ready()
        self.thread.start()

    def stop(self):
        self.should_stop = True
        self.device.stop()

    def write(self, write_command_name: str, *args):
        if write_command_name not in self.WRITE_COMMANDS:
            logger.error("{} not an available write command.".format(write_command_name))
            return
        command = self.WRITE_COMMANDS[write_command_name].encode()
        assert (len(args) == len(self.WRITE_CODES[write_command_name]),
                "%s != %s" % (len(args), len(self.WRITE_CODES[write_command_name])))

        packet = self.packet_start
        packet += str(self.write_packet_num).encode()
        packet += b"\t" + command
        for index, code in enumerate(self.WRITE_CODES[write_command_name]):
            assert self.CODE_TO_TYPE == type(args[index]), "%s != %s" % (args[index], self.CODE_TO_TYPE)
            packet += b"\t" + str(args[index]).encode()

        checksum = get_checksum(packet)
        packet += checksum.to_bytes(2, "big")
        packet += self.packet_stop

        with self.write_lock:
            self.device.write(packet)

    def parse_packet(self):
        # assuming self.device.in_waiting() returned > 0

        print_buffer = b""
        self.prev_packet_time = time.time()
        while True:
            c = self.device.read(1)
            if c == self.packet_start[0]:
                c = self.device.read(1)
                if c == self.packet_start[1]:
                    break

            if time.time() - self.prev_packet_time > device_port_config.timeout:
                raise DevicePortReadException("Timed out waiting for the packet start bytes.")
            if c == self.packet_stop:
                logger.info("Device message: " + str(print_buffer))
                print_buffer = b""
            else:
                print_buffer += c

        packet_buffer = b""
        self.prev_packet_time = time.time()
        while True:
            c = self.device.read(1)
            packet_buffer += c
            if c == self.packet_stop:
                break
            if time.time() - self.prev_packet_time > device_port_config.timeout:
                raise DevicePortReadException("Timed out waiting for the packet stop byte.")

        buffer = PacketBuffer(packet_buffer)

        if not buffer.checksum():
            logger.error(
                "Checksum failed for packet {}. Calculated {} != received {}".format(
                    packet_buffer, buffer.calc_checksum, buffer.recv_checksum)
            )

        logger.debug("read_packet_num = {}".format(buffer.packet_num))
        if self.read_packet_num != buffer.packet_num:
            logger.warn("Received packet num {} is out of sync with local packet num {}".format(
                buffer.packet_num, self.read_packet_num
            ))
            self.read_packet_num = buffer.packet_num

        if buffer.identifier not in self.data_frame:
            self.data_frame[buffer.identifier] = {"num": self.read_packet_num}
        for segment_index, segment in buffer.iter():
            subfield_name = self.PACKET_NAMES[buffer.identifier][segment_index]
            self.data_frame[buffer.identifier][subfield_name] = segment

        self.read_packet_num += 1
        return buffer.identifier

    def read_task(self, should_stop):
        update_delay = 1 / self.read_update_rate_hz
        self.prev_packet_time = time.time()

        while True:
            if should_stop():
                logger.info("Exiting read thread")
                return

            if time.time() - self.prev_time_command_update > 0.5:
                self.write_time_str()
                self.prev_time_command_update = time.time()

            in_waiting = self.device.in_waiting()
            if in_waiting:
                try:
                    recv_time = time.time()
                    identifier = self.parse_packet()

                    if identifier in self.recv_times:
                        self.recv_times[identifier].append(recv_time)
                        while len(self.recv_times[identifier]) > 0x10000:
                            self.recv_times[identifier].pop(0)
                    else:
                        self.recv_times[identifier] = [recv_time]

                    self.on_receive(identifier)
                except BaseException as e:
                    logger.error("An error occurred while waiting for a packet!", exc_info=True)
                    continue
            time.sleep(update_delay)

    def get(self, identifier, name):
        return self.data_frame[identifier][name]

    def check_ready(self):
        self.write("get_ready", "rover6")
        identifier = None
        start_time = time.time()
        prev_write_time = time.time()
        while identifier != "ready":
            if time.time() - prev_write_time > 1.0:
                self.write("get_ready", "rover6")
                prev_write_time = time.time()

            if time.time() - start_time > 25.0:
                raise DevicePortWriteException("Timed out! Failed to receive ready signal from device.")

            if self.device.in_waiting() > 0:
                try:
                    identifier = self.parse_packet()
                except DevicePortReadException:
                    logger.info("Waiting for ready signal timed out. Trying again.")
            time.sleep(0.05)

        logger.info("Device signalled ready. Rover name: {}".format(self.get(identifier, "name")))

    def set_active(self, state: bool):
        if state:
            self.write("toggle_active", 0)
        else:
            self.write("toggle_active", 1)

    def soft_restart(self):
        self.write("toggle_active", 2)

    def set_reporting(self, state: bool):
        if state:
            self.write("toggle_reporting", 0)
        else:
            self.write("toggle_reporting", 1)

    def reset_sensors(self):
        self.write("toggle_reporting", 2)

    def write_time_str(self):
        self.write("update_time_str", datetime.datetime.now().strftime("%I:%M:%S%p"))

    def set_speed(self, speed_A, speed_B):
        self.write("set_motors", float(speed_A), float(speed_B))

    def set_k(self, kp, ki, kd):
        rover_config.kp = kp
        rover_config.ki = ki
        rover_config.kd = kd
        self.write(
            "set_pid_ks",
            float(kp),
            float(ki),
            float(kd),
            float(kp),
            float(ki),
            float(kd)
        )

    def set_servo(self, n: int, command: int = None):
        if command is None:
            self.write("set_servo_default", int(n))
        else:
            self.write("set_servo", int(n), int(command))

    def on_receive(self, identifier):
        if identifier == "ina":
            print(self.get(identifier, "voltage_V"))
        elif identifier == "servo":
            print([self.get(identifier, str(index)) for index in range(4)])
            # for n in range(self.NUM_SERVOS):
            #     if self.written_servo_positions[n] is not None: 
            #         if self.written_servo_positions[n] != self.get(identifier, str(n)):
            #             self.servo_write_attempts[n] += 1
            #             if self.servo_write_attempts[n] > self.servo_write_max_attempts:
            #                 raise DevicePortWriteException("Exceeded number of attempts to write %s to servo %s" % (self.written_servo_positions[n], n))
            #             self.set_servo(n, self.written_servo_positions[n])
            #         else:
            #             self.servo_write_attempts[n] = 0

    def set_obstacle_thresholds(self, back_lower: int, back_upper: int, front_lower: int, front_upper: int):
        self.write("set_safety_thresholds", back_lower, back_upper, front_lower, front_upper)

    def set_safety_thresholds(self, obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm=100.0):
        # obstacle_threshold_x_mm: measured from the maximum point of the front or back of the robot
        # ledge_threshold_y_mm: measured from the ground plane where the robot sits flat on the ground
        # buffer_x_mm: x buffer distance to account for robot stopping time and update rate delay
        logger.info("Safety threshold settings: "
                    "obstacle_threshold_x_mm = {}, ledge_threshold_y_mm = {}, buffer_x_mm = {}".format(
            obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm)
        )

        tof_thresholds = tof_obstacles.get_stopping_thresholds(
            obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm
        )
        logger.info("Setting front safety thresholds: lower = {}, upper = {}, servo = {}".format(
            tof_thresholds.front_lower, tof_thresholds.front_upper, tof_thresholds.front_servo
        ))
        logger.info("Setting back safety thresholds: lower = {}, upper = {}, servo = {}".format(
            tof_thresholds.back_lower, tof_thresholds.back_upper, tof_thresholds.back_servo
        ))

        self.set_servo(rover_config.back_tilter_servo_num, tof_thresholds.back_servo)
        self.set_servo(rover_config.front_tilter_servo_num, tof_thresholds.front_servo)
        self.set_obstacle_thresholds(
            tof_thresholds.back_lower,
            tof_thresholds.back_upper,
            tof_thresholds.front_lower,
            tof_thresholds.front_upper,
        )

    # def __del__(self):
    #     self.stop()
