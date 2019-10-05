import datetime
from device_port import DevicePort

PACKET_CODES = {
    "ina": "lfff",
    "enc": "lll",
    "fsr": "ldd",
    "irr": "dd",
    "bno": "lfffffffff",
    "lox": "lddd"
}

def parse_packet(packet):
    fields = packet.split("\t")
    identifier = fields[0]
    data = []

    if identifier in PACKET_CODES:
        packet_code = PACKET_CODES[identifier]

        num_fields = len(fields) - 1
        if len(packet_code) != num_fields:
            if len(packet_code) < num_fields:
                print("There are %s unparsed fields: %s, '%s'. Skipping" % (len(packet_code) - num_fields, packet_code, packet))
            elif num_fields < len(packet_code):
                print("Packet code specifies %s more fields than supplied: %s, '%s'. Skipping" % (num_fields - len(packet_code), packet_code, packet))

        for index, data_type in enumerate(packet_code):
            field = fields[index + 1].decode()
            if data_type == "l" or data_type == "d":
                data.append(int(field))
            elif data_type == "s":
                data.append(field)
            elif data_type == "f":
                data.append(float(field))
            else:
                print("Invalid data type: '%s' in packet '%s'" % (data_type, packet))
    return identifier, data


usb_port = DevicePort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_5816830-if00")
uart_port = DevicePort("/dev/serial0", baud=500000)

usb_port.configure()
uart_port.configure()

uart_port.check_protocol("?", "!")
print("UART ready")

try:
    uart_port.write(">")
    while True:
        usb_waiting = usb_port.in_waiting()
        if usb_waiting:
            receive_time, packets = usb_port.read(usb_waiting)
            receive_date = datetime.datetime.fromtimestamp(receive_time)
            receive_str = datetime.datetime.strftime(receive_date, "%c")
            print("USB %s:" % (receive_str))
            for packet in packets:
                print("\t%s" % packet)

        uart_waiting = uart_port.in_waiting()
        if uart_waiting:
            receive_time, packets = uart_port.read(usb_waiting)
            receive_date = datetime.datetime.fromtimestamp(receive_time)
            receive_str = datetime.datetime.strftime(receive_date, "%c")

            for packet in packets:
                identifier, data = parse_packet(packet)
                # print("%s:\t%s" % (identifier, data))
                if identifier == "irr":
                    print("IR data: %s" % data)

except BaseException:
    uart_port.write("<")
    usb_port.close()
    uart_port.close()

    raise
