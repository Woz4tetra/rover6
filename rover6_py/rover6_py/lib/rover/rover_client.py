import time
import datetime
import threading

from . import tof_obstacles
from ..device_port import DevicePort, DevicePortReadException, DevicePortWriteException
from ..config import ConfigManager
from ..logger_manager import LoggerManager
from .packet import Packet
from ..exceptions import ShutdownException, LowBatteryException

rover_config = ConfigManager.get_rover_config()
device_port_config = ConfigManager.get_device_port_config()
logger = LoggerManager.get_logger()


class BatteryState:
    UNKNOWN = 0
    FULL = 1
    OK = 2
    LOW = 3
    CRITICAL = 4

    def __init__(self):
        self.state = 0
        self.voltage_V = 0.0
        self.prev_critical_time = None

    def set(self, voltage_V):
        self.voltage_V = voltage_V
        if voltage_V >= rover_config.full_voltage:
            self.state = self.FULL
            self.prev_critical_time = None
        elif voltage_V >= rover_config.ok_voltage:
            self.state = self.OK
            self.prev_critical_time = None
        elif voltage_V >= rover_config.low_voltage:
            self.state = self.LOW
            self.prev_critical_time = None
        elif voltage_V >= rover_config.critical_voltage:
            self.state = self.CRITICAL
            if self.prev_critical_time is None:
                self.prev_critical_time = time.time()


    def should_shutdown(self):
        return (
            self.state == self.CRITICAL and
            self.prev_critical_time is not None and
            time.time() - self.prev_critical_time > rover_config.critical_voltage_timeout_s
        )

    def log_state(self):
        if self.state == self.FULL:
            logger.info("Fully charged: %0.2f" % self.voltage_V)
        elif self.state == self.OK:
            logger.info("Battery ok: %0.2f" % self.voltage_V)
        elif self.state == self.LOW:
            logger.warn("Battery is low: %0.2f. Shutting down soon." % self.voltage_V)
        elif self.state == self.CRITICAL:
            logger.error("Battery is critically low: %0.2f!!" % self.voltage_V)
        else:
            logger.error("Battery is in an unknown state")


class RoverClient:
    PACKET_CODES = rover_config.packet_codes
    PACKET_NAMES = rover_config.packet_names
    WRITE_COMMANDS = rover_config.write_commands
    WRITE_CODES = rover_config.write_codes

    NUM_SERVOS = rover_config.num_servos

    def __init__(self):
        self.device = DevicePort(
            device_port_config.address,
            device_port_config.baud_rate,
            device_port_config.timeout,
            device_port_config.write_timeout
        )
        self.data_frame = {header: {} for header in self.PACKET_CODES.keys()}

        self.name = ""

        self.name_index_mapping = {}
        for identifier, names in self.PACKET_NAMES.items():
            self.name_index_mapping[identifier] = {name: index for index, name in enumerate(names)}

        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.thread = threading.Thread(target=self.read_task, args=(lambda: self.should_stop,))
        self.thread_exception = None

        self.recv_times = {}
        self.read_update_rate_hz = device_port_config.update_rate_hz
        self.prev_packet_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.read_packet_num = 0

        self.servo_pos = [None for _ in range(self.NUM_SERVOS)]

        # self.written_packets = {}

        self.txrx_error_codes = {
            0: "no error",
            1: "c1 != \\x12",
            2: "c2 != \\x34",
            3: "packet is too short",
            4: "checksums don't match",
            5: "packet count segment not found",
            6: "packet counts not synchronized",
            7: "failed to find category segment",
            8: "invalid format"
        }

        self.wifi_hub = None
        self.gpio_hub = None

        self.battery_state = BatteryState()
        self.prev_battery_report_time = time.time()

        self.client_start_time = None
        self.device_start_time = None

    def start(self):
        logger.info("Starting rover client")
        self.device.configure()
        logger.info("Device configured")
        self.check_ready()
        logger.info("Device ready")
        self.set_active(True)
        logger.info("Active is True")
        self.set_reporting(True)
        logger.info("Reporting is True")
        self.set_reporting(True)
        logger.info("Starting read thread")
        self.thread.start()
        time.sleep(3.0)

    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return
        logger.info("Stopping rover client")
        self.should_stop = True
        logger.info("Set read thread stop flag")
        self.set_reporting(False)
        logger.info("Reporting is False")
        time.sleep(0.01)  # make sure controller doesn't miss the next command
        self.set_active(False)
        logger.info("Active is False")
        # self.soft_restart()
        with self.read_lock:
            self.device.stop()
        logger.info("Device connection closed")

    # def remove_timedout_written_packets(self):
    #     current_time = time.time()
    #     packet_nums_to_remove = []
    #     for packet_num, written_packet in self.written_packets.items():
    #         if current_time - written_packet.timestamp > 10.0:
    #             packet_nums_to_remove.append(packet_num)
    #     if len(packet_nums_to_remove):
    #         logger.debug("Purging written packets: %s" % str(packet_nums_to_remove))
    #     for packet_num in packet_nums_to_remove:
    #         self.written_packets.pop(packet_num)

    def write(self, write_command_name: str, *args):
        packet = Packet.from_args(write_command_name, *args)
        logger.debug("Writing: %s" % str(packet))
        with self.write_lock:
            self.device.write(packet.packet)
            time.sleep(0.0005)
        # self.written_packets[packet.packet_num] = packet
        # self.remove_timedout_written_packets()

    def wait_for_packet_start(self):
        print_buffer = b""
        self.prev_packet_time = time.time()
        while True:
            c = self.device.read(1)
            if c == Packet.PACKET_START[0:1]:
                c = self.device.read(1)
                if c == Packet.PACKET_START[1:2]:
                    break  # exit the loop once the start packet is found

            if time.time() - self.prev_packet_time > device_port_config.timeout:
                raise DevicePortReadException("Timed out waiting for the packet start bytes.")
            if c == Packet.PACKET_STOP:
                message = str(print_buffer)
                level = 10
                try:
                    segments = print_buffer.decode().split("\t")
                    if len(segments) >= 3 and segments[0] == "msg":
                        if segments[1] == "INFO" or segments[1] == "ERROR":
                            message = "[%s] -- %s" % (segments[1], str("  ".join(segments[2:])))
                            if segments[1] == "INFO":
                                # level = 20
                                level = 10
                            elif segments[1] == "ERROR":
                                level = 30
                except ValueError:
                    pass

                logger.log(level, "Device message: " + message)

                print_buffer = b""
            else:
                print_buffer += c

    def parse_packet(self):
        self.wait_for_packet_start()

        packet_buffer = self.device.readline()

        logger.debug("packet_buffer: %s" % str(packet_buffer))
        packet = Packet.from_bytes(packet_buffer)

        if not packet.checksum():
            logger.error(
                "Checksum failed for packet {}. Calculated {} != received {}".format(
                    packet_buffer, packet.calc_checksum, packet.recv_checksum)
            )
            return None

        logger.debug("read_packet_num = {}, local = {}".format(packet.packet_num, self.read_packet_num))
        if self.read_packet_num != packet.packet_num:
            logger.warn("Received packet num {} is out of sync with local packet num {}".format(
                packet.packet_num, self.read_packet_num
            ))
            self.read_packet_num = packet.packet_num

        if packet.identifier not in self.data_frame:
            self.data_frame[packet.identifier] = {"num": self.read_packet_num, "recv_time": 0.0}
        for segment_index, segment in packet.iter():
            subfield_name = self.PACKET_NAMES[packet.identifier][segment_index]
            self.data_frame[packet.identifier][subfield_name] = segment

        self.read_packet_num += 1
        return packet

    # def on_device_packet_status(self):
    #     if not self.on_txrx():
    #         packet_num = self.get("txrx", "packet_num")
    #         if packet_num in self.written_packets:
    #             sent_packet = self.written_packets[packet_num]
    #             logger.info("Device failed to receive packet num %s, identifier = %s. Re-transmitting." % (
    #                 packet_num, sent_packet.identifier))
    #             new_packet = Packet.from_packet(sent_packet)
    #             self.device.write(new_packet.packet)
    #         else:
    #             logger.error("Device failed to receive packet num %s, but we never sent it!" % packet_num)
    #             logger.info("Written packets: %s" % str(self.written_packets))

    def on_recv_time(self, identifier):
        recv_time = self.get(identifier, "time")
        if recv_time is None:
            self.data_frame[identifier]["recv_time"] = time.time()
            return
        
        recv_time *= 1E-3  # ms to s

        # recv_time = time.time()
        if self.client_start_time is None:
            self.client_start_time = time.time()
        if self.device_start_time is None:
            self.device_start_time = recv_time

        self.data_frame[identifier]["recv_time"] = self.client_start_time + (recv_time - self.device_start_time)

        if recv_time:
            if identifier in self.recv_times:
                self.recv_times[identifier].append(recv_time)
                while len(self.recv_times[identifier]) > 0x10000:
                    self.recv_times[identifier].pop(0)
            else:
                self.recv_times[identifier] = [recv_time]

    def read_task_running(self):
        return self.thread_exception is None

    def read_task(self, should_stop):
        update_delay = 1 / self.read_update_rate_hz
        self.prev_packet_time = time.time()

        try:
            while True:
                time.sleep(update_delay)
                if should_stop():
                    logger.info("Exiting read thread")
                    return

                if time.time() - self.prev_time_command_update > rover_config.update_rpi_state_delay:
                    self.update_rpi_state()
                    self.prev_time_command_update = time.time()

                if self.device.in_waiting() == 0:
                    continue
                # try:
                with self.read_lock:
                    packet = self.parse_packet()
                # except DevicePortReadException as e:
                #     logger.error("%s occurred while waiting for packet! Exiting." % str(e), exc_info=True)
                #     self.thread_exception = e
                #     break

                self.on_receive(packet.identifier)
        except BaseException as e:
            logger.error("An exception occurred in the read thread", exc_info=True)
            self.thread_exception = e

    def _get_from_name(self, identifier, name):
        if identifier in self.data_frame:
            if name in self.data_frame[identifier]:
                return self.data_frame[identifier][name]
        return None

    def get(self, identifier, *names):
        if len(names) > 1:
            result = []
            for name in names:
                result.append(self._get_from_name(identifier, name))
            result = tuple(result)
        elif len(names) == 1:
            result = self._get_from_name(identifier, names[0])
        else:
            return None

        return result

    def check_ready(self):
        self.write("get_ready", "rover6")
        identifier = None
        start_time = time.time()
        prev_write_time = time.time()
        packet = None
        while identifier != "ready":
            if time.time() - prev_write_time > 1.0:
                self.write("get_ready", "rover6")
                prev_write_time = time.time()

            if time.time() - start_time > 25.0:
                raise DevicePortWriteException("Timed out! Failed to receive ready signal from device.")

            if self.device.in_waiting() > 0:
                try:
                    with self.read_lock:
                        packet = self.parse_packet()
                        identifier = packet.identifier
                except DevicePortReadException:
                    logger.info("Waiting for ready signal timed out. Trying again.")

                if packet:
                    if identifier == "txrx":
                        self.on_txrx()
                    else:
                        logger.info(identifier, self.data_frame[packet.identifier])
            time.sleep(0.05)

        self.name = self.get(identifier, "name")
        logger.info("Device signalled ready. Rover name: {}".format(self.name))

    def set_active(self, state: bool):
        if state:
            self.write("toggle_active", 1)
        else:
            self.write("toggle_active", 0)

    def soft_restart(self):
        self.write("toggle_active", 2)

    def set_reporting(self, state: bool):
        if state:
            self.write("toggle_reporting", 1)
        else:
            self.write("toggle_reporting", 0)

    def reset_sensors(self):
        self.write("toggle_reporting", 2)

    def set_wifi_hub(self, wifi_hub):
        self.wifi_hub = wifi_hub

    def set_gpio_hub(self, gpio_hub):
        self.gpio_hub = gpio_hub

    def update_rpi_state(self):
        self.write(
            "rpi_state",
            self.wifi_hub.ip_address,
            self.wifi_hub.hostname,
            datetime.datetime.now().strftime("%I:%M:%S%p"),
            int(self.gpio_hub.button_state),
            self.wifi_hub.status_code
        )

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
        if self.servo_pos[n] == command:
            return
        self.servo_pos[n] = command
        if command is None:
            self.write("set_servo_default", int(n))
        else:
            self.write("set_servo", int(n), int(command))

    def on_receive(self, identifier):
        self.on_recv_time(identifier)

        if identifier == "ina":
            voltage_V = self.get(identifier, "voltage_V")
            self.battery_state.set(voltage_V)
            if time.time() - self.prev_battery_report_time > rover_config.battery_log_report_time:
                self.battery_state.log_state()
                self.prev_battery_report_time = time.time()
            if self.battery_state.should_shutdown():
                raise LowBatteryException("Battery is critically low: %0.2f!! Shutting down." % voltage_V)
        elif identifier == "shutdown":
            if self.get(identifier, "name") == "rover6":
                raise ShutdownException("Device requested a shutdown.")
        elif identifier == "txrx":
            self.on_txrx()

    def on_txrx(self):
        packet_num = self.get("txrx", "packet_num")
        success = self.get("txrx", "success")
        if success != 0:
            logger.warn("Serial packet #%s error %s: %s" % (packet_num, success, self.txrx_error_codes[success]))
        return success == 0

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
            int(tof_thresholds.front_upper),
            int(tof_thresholds.back_upper),
            int(tof_thresholds.front_lower),
            int(tof_thresholds.back_lower),
        )

    # def __del__(self):
    #     self.stop()
