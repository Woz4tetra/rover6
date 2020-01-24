#ifndef ROVER6_SERIAL
#define ROVER6_SERIAL

#include <Arduino.h>

#define DATA_SERIAL  Serial5
#define SERIAL_MSG_BUFFER_SIZE 0xff
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
#define PACKET_END '\n'
bool is_reporting_enabled = false;
String data_buffer = "";
String prev_packet = "";
String current_packet = "";

void process_serial_packet(String packet);


void read_all_serial()
{
    if (!DATA_SERIAL.available()) {
        return;
    }
    size_t counter = 0;
    while (DATA_SERIAL.available()) {
        current_packet = DATA_SERIAL.readStringUntil('\n');
        process_serial_packet(current_packet);
        counter++;
        if (counter > 0x10000) {
            break;
        }
    }
    // char c;
    // size_t index = 0;
    // current_packet = prev_packet;
    // prev_packet = "";
    // while (DATA_SERIAL.available()) {
    //     c = DATA_SERIAL.read();
    //     if (c == '\n') {
    //         process_serial_packet(current_packet);
    //         current_packet = "";
    //     }
    //     else {
    //         current_packet += String(c);
    //     }
    //     index++;

    //     if (index > 0x100) {
    //         break;
    //     }
    // }

    // if (current_packet.length() > 0) {
    //     prev_packet = current_packet;
    // }
}

void print_data(String name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    String data = String(formats) + "\t";
    while (*formats != '\0') {
        if (*formats == 'd') {
            int i = va_arg(args, int);
            data += String(i);
        }
        else if (*formats == 'l') {
            int32_t s = va_arg(args, int32_t);
            data += String(s);
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            data += s;
        }
        else if (*formats == 'f') {
            double f = va_arg(args, double);
            data += String(f);
        }
        data += "\t";
        ++formats;
    }
    va_end(args);
    data += PACKET_END;
    DATA_SERIAL.print(name + "\t" + data);
}


void print_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tINFO\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tINFO\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void print_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tERROR\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tERROR\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void setup_serial()
{
    // DATA_SERIAL.begin(115200);
    DATA_SERIAL.begin(500000);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
    println_info("Rover #6");
    println_info("Serial buses initialized.");
}

#endif // ROVER6_SERIAL
