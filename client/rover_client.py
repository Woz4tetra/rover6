import time
import datetime
import threading
from device_port import DevicePort


class RoverClient:

    PACKET_CODES = {
        "ina": "lfff",
        "enc": "lllff",
        "fsr": "ldd",
        "irr": "ldd",
        "bno": "lfffffffff",
        "lox": "ldddddd",
        "servo": "ldddddddddddddddd",
    }

    PACKET_NAMES = {
        "ina":  ["time", "current_mA", "power_mW", "voltage_V"],
        "enc":  ["time", "posA", "posB", "speedA", "speedB"],
        "fsr":  ["time", "left", "right"],
        "irr":  ["time", "type", "value"],
        "bno":  ["time", "orientation_x", "orientation_y", "orientation_z", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"],
        "lox":  ["time", "front_mm", "back_mm", "front_measure_status", "back_measure_status", "front_status", "back_status"],
        "servo": ["time"] + [str(i) for i in range(16)],
    }
    
    def __init__(self):
        self.device = DevicePort("/dev/serial0", baud=500000)
        self.data_frame = {header: None for header in self.PACKET_CODES.keys()}
        self.name_index_mapping = {}
        for header, names in self.PACKET_NAMES.items():
            self.name_index_mapping[header] = {name: index for index, name in enumerate(names)}
        
        self.should_stop = False
        self.thread = threading.Thread(target=self.update, args=(lambda: self.should_stop,))

    def start(self):
        self.device.configure()
        self.device.check_protocol("?", "!")
        print("Teensy ready")
        self.write(">")  # bring out of standby
        self.write("]")  # start reporting data
        self.thread.start()
    
    def stop(self):
        self.should_stop = True
    
    def write(self, packet):
        if len(packet) == 0:
            return
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
        return identifier, data

    def update(self, should_stop):
        while True:
            if should_stop():
                print("Exiting read thread")
                return

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
    
    def on_receive(self, identifier):
        if identifier == "ina":
            print(self.get(identifier, "voltage_V"))

    def get(self, identifier, name):
        return self.data_frame[identifier][1][self.name_index_mapping[identifier][name]]

    def set_speed(self, speed_A, speed_B):
        self.write("ma%0.2f" % speed_A)
        self.write("mb%0.2f" % speed_B)

    def set_k(self, kp, ki, kd):
        self.write("kap%s" % kp)
        self.write("kai%s" % ki)
        self.write("kad%s" % kd)
        self.write("kbp%s" % kp)
        self.write("kbi%s" % ki)
        self.write("kbd%s" % kd)
    


    def __del__(self):
        self.stop()


