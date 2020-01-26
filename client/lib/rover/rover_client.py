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
    checksum &= 0xff
    return checksum


class PacketBuffer:
    def __init__(self, packet: bytes):
        self.packet = packet
        self.index = 0
        self.recv_checksum = 0
        self.calc_checksum = 0

        self.type_mapping = {
            'd': self.read_int,
            'i': self.read_int,
            'l': self.read_long,
            'f': self.read_float,
        }

    def read(self, n) -> bytes:
        segment = self.packet[self.index: self.index + n]
        self.index += n
        return segment

    def read_int(self) -> int:
        return int.from_bytes(self.read(4), "big")

    def read_float(self) -> float:
        return struct.unpack('f', self.read(4))[0]

    def read_long(self) -> int:
        return int.from_bytes(self.read(8), "big")

    def read_until(self, c) -> bytes:
        found_index = self.packet.find(c)
        if found_index == -1:
            segment = self.packet[self.index:]
            self.index = len(self.packet)
        else:
            segment = self.packet[self.index:found_index]
            self.index = found_index
        return segment

    def checksum(self) -> bool:
        self.recv_checksum = int.from_bytes(self.packet[-2:], "big")
        self.calc_checksum = get_checksum(self.packet[:-2])
        return self.recv_checksum == self.calc_checksum

    def iter(self):
        subfield_index = 0
        while self.index < len(self.packet) - 2:  # ignore checksum bytes
            subfield_type = self.read(1)
            if subfield_type == 'c':
                yield subfield_index, self.read(1)
            elif subfield_type == 's':
                yield subfield_index, self.read_until(b'\x00')
            else:
                yield subfield_index, self.type_mapping[subfield_type]()
            subfield_index += 1


class RoverClient:
    PACKET_CODES = {
        "serial": 1,
        "bno"   : 2,
        "enc"   : 3,
        "fsr"   : 4,
        "safety": 5,
        "ina"   : 6,
        "ir"    : 7,
        "servo" : 8,
        "tof"   : 9,
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
        "serial": ["time", "name"],
    }

    WRITE_CODES = {
        "toggle_active"        : 1,
        "get_ready"            : 2,
        "toggle_reporting"     : 3,
        "update_time_str"      : 4,
        "set_motors"           : 5,
        "set_pid_ks"           : 6,
        "set_servo"            : 7,
        "set_servo_default"    : 8,
        "set_safety_thresholds": 9
    }

    def __init__(self):
        self.device = DevicePort()
        self.data_frame = {header: None for header in self.PACKET_CODES.keys()}

        self.name_index_mapping = {}
        for identifier, names in self.PACKET_NAMES.items():
            self.name_index_mapping[identifier] = {name: index for index, name in enumerate(names)}

        self.packet_code_index_mapping = {index: identifier for identifier, index in self.PACKET_CODES.items()}

        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.thread = threading.Thread(target=self.read_task, args=(lambda: self.should_stop,))

        self.recv_times = {}
        self.read_update_rate_hz = 120.0
        self.prev_packet_time = 0.0

        self.written_servo_positions = [None for i in range(self.NUM_SERVOS)]
        self.servo_write_attempts = [0 for i in range(self.NUM_SERVOS)]
        self.servo_write_max_attempts = 60
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

    def create_packet(self, category: int, *args):
        message = b""
        for arg in args:
            if type(arg) == int:
                if arg > 0xffffffff:  # 32 bit int
                    subfield_type = b'l'
                    subfield = arg.to_bytes(8, "big")
                else:
                    subfield_type = b'i'
                    subfield = arg.to_bytes(4, "big")
            elif type(arg) == float:
                subfield_type = b'f'
                subfield = struct.pack('f', arg)
            elif type(arg) == str:
                if len(arg) == 1:
                    subfield_type = 'c'
                else:
                    subfield_type = 's'
                subfield = arg.encode()
            else:
                logger.error("Invalid argument supplied: {}. Must be int, float, or str".format(arg))
                return None
            message += subfield_type + subfield

        category_bytes = category.to_bytes(1, "big")
        write_packet_num_bytes = self.write_packet_num.to_bytes(4, "big")
        message = write_packet_num_bytes + category_bytes + message

        checksum = get_checksum(message)
        checksum_bytes = checksum.to_bytes(2, "big")
        msg_length = (len(message) + 2).to_bytes(2, "big")

        packet = self.packet_start + msg_length + message + checksum_bytes

        self.write_packet_num += 1

        return packet

    def write(self, category: str, *args):
        if category not in self.WRITE_CODES:
            logger.error("{} not an available write code.".format(category))
            return
        with self.write_lock:
            packet = self.create_packet(self.WRITE_CODES[category], *args)
            self.device.write(packet)

    def parse_packet(self):
        # assuming self.device.in_waiting() returned > 0

        print_buffer = b""
        while True:
            c = self.device.read(1)
            if c == self.packet_start[0]:
                c = self.device.read(1)
                if c == self.packet_start[1]:
                    break

            if time.time() - self.prev_packet_time > device_port_config.timeout:
                raise DevicePortReadException("Timed out waiting for the packet start bytes.")
            if c == self.packet_stop:
                logger.info("Device message: " + print_buffer.decode())
                print_buffer = b""
            else:
                print_buffer += c

        msg_length_bytes = self.device.read(2)
        logger.debug("msg_length_bytes = {}".format(msg_length_bytes))
        msg_length = int.from_bytes(msg_length_bytes, byteorder="big")
        logger.debug("msg_length = {}".format(msg_length))
        packet = self.device.read(msg_length + 1)  # include newline
        if packet[-1] != self.packet_stop:
            raise DevicePortReadException("Packet {} does not end with terminating character!".format(packet))

        packet = packet[:-1]
        buffer = PacketBuffer(packet)

        if not buffer.checksum():
            logger.error(
                "Checksum failed for packet {}. Calculated {} != received {}".format(
                    packet, buffer.calc_checksum, buffer.recv_checksum)
            )

        read_packet_num = buffer.read_int()
        logger.debug("read_packet_num = {}".format(read_packet_num))
        if self.read_packet_num != read_packet_num:
            logger.warn("Received packet num {} is out of sync with local packet num {}".format(
                read_packet_num, self.read_packet_num
            ))
            self.read_packet_num = read_packet_num
        self.read_packet_num += 1

        category = int.from_bytes(buffer.read(1), "big")
        logger.debug("category = {}".format(category))

        identifier = self.packet_code_index_mapping[category]
        for subfield_index, subfield in buffer.iter():
            if identifier not in self.data_frame:
                self.data_frame[identifier] = {"num": read_packet_num}
            subfield_name = self.PACKET_NAMES[identifier][subfield_index]
            self.data_frame[identifier][subfield_name] = subfield

        return identifier

    def read_task(self, should_stop):
        update_delay = 1 / self.read_update_rate_hz
        self.prev_packet_time = time.time()

        while True:
            if should_stop():
                print("Exiting read thread")
                return

            if time.time() - self.prev_time_command_update > 0.5:
                self.write_time_str()
                self.prev_time_command_update = time.time()

            in_waiting = self.device.in_waiting()
            if in_waiting:
                try:
                    identifier = self.parse_packet()

                    if identifier in self.recv_times:
                        self.recv_times[identifier].append(time.time())
                        while len(self.recv_times[identifier]) > 0x10000:
                            self.recv_times[identifier].pop(0)
                    else:
                        self.recv_times[identifier] = [time.time()]

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
        while identifier != "serial":
            if time.time() - prev_write_time > 1.0:
                self.write("get_ready", "rover6")
                prev_write_time = time.time()

            if time.time() - start_time > 5.0:
                raise DevicePortWriteException("Timed out! Failed to receive ready signal from device.")

            identifier = self.parse_packet()

        logger.info("Device signalled ready. Rover name: {}".format(self.get(identifier, "name")))

    def set_active(self, state: bool):
        if state:
            self.write("toggle_active", chr(1))
        else:
            self.write("toggle_active", chr(2))

    def soft_restart(self):
        self.write("toggle_active", chr(3))

    def set_reporting(self, state: bool):
        if state:
            self.write("toggle_reporting", chr(1))
        else:
            self.write("toggle_reporting", chr(2))

    def reset_sensors(self):
        self.write("toggle_reporting", chr(3))

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

    def format_servo_command(self, n: int, command: int = None):
        if command is None:
            return int(n) & 0xff
        else:
            return (int(n) & 0xff) << 8 | (int(command) & 0xff)

    def set_servo(self, n: int, command: int = None):
        self.write("set_servo_defaults" if command is None else "set_servo", self.format_servo_command(n, command))
        self.written_servo_positions[n] = command

    def set_servos(self, commands: dict):
        args = []
        for n, command in commands.items():
            if command is None:
                raise ValueError("Can't set servo to default position with this command.")
            args.append(self.format_servo_command(n, command))
        self.write("set_servo", *args)

    def set_servo_defaults(self, *servo_nums):
        args = []
        for n in servo_nums:
            args.append(self.format_servo_command(n))
        self.write("set_servo_defaults", *args)

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
        servo_commands = {
            rover_config.back_tilter_servo_num : tof_thresholds.back_servo,
            rover_config.front_tilter_servo_num: tof_thresholds.front_servo
        }

        logger.info("Setting front safety thresholds: lower = {}, upper = {}, servo = {}".format(
            tof_thresholds.front_lower, tof_thresholds.front_upper, tof_thresholds.front_servo
        ))
        logger.info("Setting back safety thresholds: lower = {}, upper = {}, servo = {}".format(
            tof_thresholds.back_lower, tof_thresholds.back_upper, tof_thresholds.back_servo
        ))

        self.set_servos(servo_commands)
        self.set_obstacle_thresholds(
            tof_thresholds.back_lower,
            tof_thresholds.back_upper,
            tof_thresholds.front_lower,
            tof_thresholds.front_upper,
        )

    def __del__(self):
        self.stop()
