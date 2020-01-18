import time
import datetime
from device_port import DevicePort

PACKET_CODES = {
    "ina": "lfff",
    "enc": "lllff",
    "fsr": "ldd",
    "irr": "ldd",
    "bno": "lfffffffff",
    "lox": "ldddddd"
}

def parse_packet(packet):
    fields = packet.split("\t")
    if len(fields) <= 2:
        return
    identifier = fields[0]
    data = []

    if identifier in PACKET_CODES:
        fields = fields[:-1]
        packet_code = PACKET_CODES[identifier]

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

def set_speed(speed):
    uart_port.write("ma%s\n" % speed)
    uart_port.write("mb%s\n" % speed)

def set_k(kp, ki, kd):
    uart_port.write("kap%s\n" % kp)
    uart_port.write("kai%s\n" % ki)
    uart_port.write("kad%s\n" % kd)
    uart_port.write("kbp%s\n" % kp)
    uart_port.write("kbi%s\n" % ki)
    uart_port.write("kbd%s\n" % kd)


# usb_port = DevicePort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_5816830-if00")
uart_port = DevicePort("/dev/serial0", baud=500000)

# usb_port.configure()
uart_port.configure()

uart_port.check_protocol("?", "!")
print("UART ready")

data_frame = {}

try:
    uart_port.write(">")
    uart_port.write("]")
    set_k(0.75, 0, 0.05)

    # for i in range(0, 50, 5):
    #     uart_port.write("ma%s\n" % i)
    #     uart_port.write("mb%s\n" % i)
    #     time.sleep(0.25)
    #     print("speed:", i)

    prev_time = time.time()
    while True:
        if time.time() - prev_time > 0.5:
            set_speed(150)
            prev_time = time.time()
        
        uart_waiting = uart_port.in_waiting()
        if uart_waiting:
            receive_time, packets = uart_port.read(uart_waiting)
            receive_date = datetime.datetime.fromtimestamp(receive_time)
            receive_str = datetime.datetime.strftime(receive_date, "%c")

            for packet in packets:
                result = parse_packet(packet)
                if result is None:
                    continue
                identifier, data = result
                # print("%s:\t%s" % (identifier, data))

                data_frame[identifier] = data
                if identifier == "irr":
                    print("data: %s" % [hex(x) for x in data[1:]])
                    # for i, d in data_frame.items():
                    #     print("%s:\t%s" % (i, d))
                elif identifier == "enc":
                    # print("%s:\t%s" % (identifier, data))
                    print("speed; a: {3}, b: {4}".format(*data))
                elif identifier == "msg":
                    print(data[0], data[1])



except BaseException:
    uart_port.write("[")
    uart_port.write("<")
    uart_port.stop()

    raise
