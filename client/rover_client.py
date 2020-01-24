import math
import time
import datetime
import threading
from device_port import DevicePort

from rover_config import RoverConfig
import tof_obstacles

class RoverClient:

    PACKET_CODES = {
        "ina": "lfff",
        "enc": "lllff",
        "fsr": "ldd",
        "irr": "ldd",
        "bno": "lfffffffffd",
        "lox": "ldddddd",
        "servo": "ldddddddddddddddd",
        "safe": "ldddddddddddd"
    }

    NUM_SERVOS = 16

    PACKET_NAMES = {
        "ina":  ["time", "current_mA", "power_mW", "voltage_V"],
        "enc":  ["time", "posA", "posB", "speedA", "speedB"],
        "fsr":  ["time", "left", "right"],
        "irr":  ["time", "type", "value"],
        "bno":  ["time", "orientation_x", "orientation_y", "orientation_z", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z", "temperature"],
        "lox":  ["time", "front_mm", "back_mm", "front_measure_status", "back_measure_status", "front_status", "back_status"],
        "servo": ["time"] + [str(i) for i in range(NUM_SERVOS)],
        "safe": ["time", "is_left_bumper_trig", "is_right_bumper_trig", "is_front_tof_trig", "is_back_tof_trig", "is_front_tof_ok", "is_back_tof_ok", "are_servos_active", "are_motors_active", "voltage_ok", "is_active", "is_reporting_enabled", "is_speed_pid_enabled"],
    }
    
    def __init__(self):
        self.device = DevicePort("/dev/serial0", baud=500000)
        self.data_frame = {header: None for header in self.PACKET_CODES.keys()}
        self.name_index_mapping = {}
        for header, names in self.PACKET_NAMES.items():
            self.name_index_mapping[header] = {name: index for index, name in enumerate(names)}
        
        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.thread = threading.Thread(target=self.update, args=(lambda: self.should_stop,))

        self.recv_times = {}
        self.read_update_rate_hz = 120.0

        self.written_servo_positions = [None for i in range(self.NUM_SERVOS)]
        self.servo_write_attempts = [0 for i in range(self.NUM_SERVOS)]
        self.servo_write_max_attempts = 60

    def start(self):
        self.device.configure()
        self.device.check_protocol("?", "!")
        print("Teensy ready")
        self.write(">")  # bring out of standby
        self.write("]")  # start reporting data
        self.thread.start()
    
    def stop(self):
        self.write("[")
        self.write("<")
        self.should_stop = True
    
    def write(self, packet):
        if len(packet) == 0:
            return
        print("writing: ", packet)
        self.device.write(packet)
        # time.sleep(0.001)
    
    def parse_packet(self, packet):
        fields = packet.split("\t")
        if len(fields) <= 2:
            return
        identifier = fields[0]
        data = []

        if identifier in self.PACKET_CODES:
            fields = fields[:-1]
            packet_code = self.PACKET_CODES[identifier]

            num_fields = len(fields) - 2
            if len(packet_code) != num_fields:
                if len(packet_code) < num_fields:
                    print("There are %s unparsed fields: %s, '%s'. Skipping" % (len(packet_code) - num_fields, packet_code, packet))
                elif num_fields < len(packet_code):
                    print("Packet code specifies %s more fields than supplied: %s, '%s'. Skipping" % (num_fields - len(packet_code), packet_code, packet))

            for index, data_type in enumerate(packet_code):
                field = fields[index + 2]
                if data_type == "l" or data_type == "d":
                    data.append(int(field))
                elif data_type == "s":
                    data.append(field)
                elif data_type == "f":
                    data.append(float(field))
                else:
                    print("Invalid data type: '%s' in packet '%s'" % (data_type, packet))
        elif identifier == "msg":
            data.append(fields[1])
            data.append(fields[2].strip())
        else:
            print("Unparsed packet:", packet)
        return identifier, data

    def update(self, should_stop):
        update_delay = 1 / self.read_update_rate_hz
        while True:
            if should_stop():
                print("Exiting read thread")
                return
            
            if time.time() - self.prev_time_command_update > 0.5:
                self.write("t" + datetime.datetime.now().strftime("%I:%M:%S%p"))
                self.prev_time_command_update = time.time()

            in_waiting = self.device.in_waiting()
            if in_waiting:
                receive_time, packets = self.device.read(in_waiting)

                for packet in packets:
                    try:
                        result = self.parse_packet(packet)
                    except BaseException as e:
                        print("%s: %s" % (e.__class__.__name__, e))
                        continue
                    if result is None:
                        continue
                    identifier, data = result
                    self.data_frame[identifier] = receive_time, data
                    self.on_receive(identifier)

                    if identifier == "msg":
                        receive_date = datetime.datetime.fromtimestamp(receive_time)
                        receive_str = datetime.datetime.strftime(receive_date, "%c")
                        print("%s\t%s\t%s" % (receive_str, data[0], data[1]))
            time.sleep(update_delay)

    def get(self, identifier, name):
        return self.data_frame[identifier][1][self.name_index_mapping[identifier][name]]

    def set_speed(self, speed_A, speed_B):
        self.write("ma%0.2f" % speed_A)
        self.write("mb%0.2f" % speed_B)

    def set_k(self, kp, ki, kd):
        RoverConfig.kp = kp
        RoverConfig.ki = ki
        RoverConfig.kd = kd
        self.write("kap%s" % float(kp))
        self.write("kai%s" % float(ki))
        self.write("kad%s" % float(kd))
        self.write("kbp%s" % float(kp))
        self.write("kbi%s" % float(ki))
        self.write("kbd%s" % float(kd))
    
    def tell_servo(self):
        self.write("st")

    def set_servo(self, n, command):
        self.write("sp%02d%03d" % (int(n), int(command)))
        self.written_servo_positions[n] = command
        self.servo_write_attempts[n] += 1
        time.sleep(0.01)
        self.tell_servo()
    
    def on_receive(self, identifier):
        if identifier in self.recv_times:
            self.recv_times[identifier].append(time.time())
            while len(self.recv_times[identifier]) > 0x10000:
                self.recv_times[identifier].pop(0)
        else:
            self.recv_times[identifier] = [time.time()]
        
        if identifier == "ina":
            print(self.get(identifier, "voltage_V"))
        elif identifier == "servo":
            print([self.get(identifier, str(index)) for index in range(4)])
            for n in range(self.NUM_SERVOS):
                if self.written_servo_positions[n] is not None and self.written_servo_positions[n] != self.get(identifier, str(n)):
                    self.servo_write_attempts[n] += 1
                    if self.servo_write_attempts[n] > self.servo_write_max_attempts:
                        raise DevicePortWriteException("Exceeded number of attempts to write %s to servo %s" % (self.written_servo_positions[n], n))
                    self.set_servo(n, self.written_servo_positions[n])


    
    def set_obstacle_thresholds(self, lower, upper, direction: int):
        # direction: 0 = front, 1 = Back
        direction_key = 'f' if direction == 0 else 'b'
        self.write("ll%s%s" % (direction_key, int(lower)))
        self.write("lu%s%s" % (direction_key, int(upper)))

    def set_safety_thresholds(self, obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm=100.0):
        # obstacle_threshold_x_mm: measured from the maximum point of the front or back of the robot
        # ledge_threshold_y_mm: measured from the ground plane where the robot sits flat on the ground
        # buffer_x_mm: x buffer distance to account for robot stopping time and update rate delay

        tof_thresholds = tof_obstacles.get_stopping_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm)
        self.set_servo(RoverConfig.front_tilter_servo_num, tof_thresholds.front_servo)
        self.set_obstacle_thresholds(tof_thresholds.front_lower, tof_thresholds.front_upper, 0)
        print("front:", tof_thresholds.front_lower, tof_thresholds.front_upper, tof_thresholds.front_servo)

        self.set_servo(RoverConfig.back_tilter_servo_num, tof_thresholds.back_servo)
        self.set_obstacle_thresholds(tof_thresholds.back_lower, tof_thresholds.back_upper, 1)
        print("back:", tof_thresholds.back_lower, tof_thresholds.back_upper, tof_thresholds.back_servo)


    def __del__(self):
        self.stop()


