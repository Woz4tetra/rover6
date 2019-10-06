import io
import time
import datetime
from PIL import Image
import numpy as np
from device_port import DevicePort

def color565(red, green, blue):
    # uint16_t Adafruit_SPITFT::color565(uint8_t red, uint8_t green, uint8_t blue) {
    #     return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
    # }
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)


usb_port = DevicePort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_5816830-if00")
uart_port = DevicePort("/dev/serial0", baud=500000)

usb_port.configure()
uart_port.configure()

uart_port.check_protocol("?", "!")
print("UART ready")


width, height = 160, 128
image = Image.open("cam.jpg")
thumb_image = image.resize((width, height))
thumb_image = thumb_image.transpose(Image.ROTATE_90)

buf = io.BytesIO()
thumb_image.save(buf, format='BMP')
byte_im = buf.getvalue()
# file_len = len(byte_im)
# file_len_bytes = file_len.to_bytes(2, byteorder='big')
byte_im = b'd' + byte_im + b'\n'
# im_array = np.asarray(image)
# tft_array = []
# for row in range(height):
#     for col in range(width):
#         red = im_array[row][col][0]
#         green = im_array[row][col][1]
#         blue = im_array[row][col][2]
#         tft_color = color565(red, green, blue)
#         lower = tft_color & 0xff
#         upper = tft_color >> 8
#         tft_array.append(lower)
#         tft_array.append(upper)
# tft_bytes = bytes(tft_array)
# tft_bytes = b'd' + tft_bytes + b'\n'


try:
    uart_port.write(">")
    time.sleep(0.1)
    print("Sending image")
    uart_port.device.write(byte_im)
    # for index in range(len(byte_im)):
    #     uart_port.device.write(byte_im[index: index + 1])
    #     if index % 100 == 0:
    #         time.sleep(0.01)
    #         print(index)

    port = usb_port
    while True:
        # for port in [uart_port, usb_port]:
        waiting = port.in_waiting()
        if waiting:
            receive_time, packets = port.read(waiting)
            receive_date = datetime.datetime.fromtimestamp(receive_time)
            receive_str = datetime.datetime.strftime(receive_date, "%c")
            print("%s:" % (receive_str))
            for packet in packets:
                print("\t%s" % packet)

except BaseException:
    uart_port.write("<")
    usb_port.stop()
    uart_port.stop()

    raise
