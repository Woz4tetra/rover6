import datetime
from PIL import Image
import numpy as np
from device_port import DevicePort

usb_port = DevicePort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_5816830-if00")
uart_port = DevicePort("/dev/serial0", baud=500000)

usb_port.configure()
uart_port.configure()

uart_port.check_protocol("?", "!")
print("UART ready")

uart_port.write(">")

def color565(red, green, blue):
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
# uint16_t Adafruit_SPITFT::color565(uint8_t red, uint8_t green, uint8_t blue) {
#     return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
# }

width, height = 128, 64
image = Image.open("cam.jpg")
image = image.resize((width, height))
im_array = np.asarray(image)
tft_array = []
for row in range(height):
    for col in range(width):
        red = im_array[row][col][0]
        green = im_array[row][col][1]
        blue = im_array[row][col][2]
        tft_color = color565(red, green, blue)
        lower = tft_color >> 8
        upper = tft_color & 0xff
        tft_array.append(lower)
        tft_array.append(upper)

tft_bytes = bytes(tft_array)
tft_bytes = b'd' + tft_bytes + b'\n'

uart_port.device.write(tft_bytes)

try:
    uart_port.write(">")
    while True:
        for port in [uart_port, usb_port]:
            waiting = usb_port.in_waiting()
            if waiting:
                receive_time, packets = port.read(usb_waiting)
                receive_date = datetime.datetime.fromtimestamp(receive_time)
                receive_str = datetime.datetime.strftime(receive_date, "%c")
                print("%s:" % (receive_str))
                for packet in packets:
                    print("\t%s" % packet)

except BaseException:
    uart_port.write("<")
    usb_port.close()
    uart_port.close()

    raise
