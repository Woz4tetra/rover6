#ifndef ROVER6_SERIAL
#define ROVER6_SERIAL

#include <Arduino.h>

#define DATA_SERIAL  Serial5
#define SERIAL_MSG_BUFFER_SIZE 0xff
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
#define PACKET_START_0 '\x12'
#define PACKET_START_1 '\x34'
#define PACKET_STOP '\n'
bool is_reporting_enabled = false;
String data_buffer = "";
String prev_packet = "";
String current_packet = "";
String current_segment = "";
uint32_t read_packet_num = 0;
uint32_t write_packet_num = 0;
unsigned int current_packet_index = 0;

void print_data(String name, const char *formats, ...);
void print_info(const char* message, ...);
void println_info(const char* message, ...);
void print_error(const char* message, ...);
void println_error(const char* message, ...);

void process_serial_packet(String category, String packet);


bool get_segment(String packet, String* segment, unsigned int* index)
{
    int separator = packet.indexOf('\t');
    if (separator < 0) {
        *segment = packet.substring(*index);
        *index = packet.length();
        return false;
    }
    else {
        *segment = packet.substring(*index, separator);
        *index += separator + 1;
        return true;
    }
}


bool read_one_packet()
{
    char c1 = DATA_SERIAL.read();
    while (!DATA_SERIAL.available());
    if (c1 != PACKET_START_0) {
        return false;
    }
    char c2 = DATA_SERIAL.read();
    if (c2 != PACKET_START_1) {
        return false;
    }

    uint16_t calc_checksum = 0;

    current_packet = DATA_SERIAL.readStringUntil('\n');

    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (current_packet.length() < 5) {
        print_data("txrx", "dd", read_packet_num, 0);
        read_packet_num++;
        return true;
    }

    current_packet_index = 0;

    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < current_packet.length() - 2; index++) {
        calc_checksum += (uint16_t)current_packet.charAt(index);
    }

    String checksum_str = current_packet.substring(current_packet.length() - 2);
    uint16_t recv_checksum = checksum_str.charAt(0) << 8 | checksum_str.charAt(1);

    // checksum failed, but we read one packet
    if (calc_checksum != recv_checksum) {
        print_data("txrx", "dd", read_packet_num, 0);
        read_packet_num++;
        return true;
    }
    // failed to find packet num segment, but we read one packet
    if (get_segment(current_packet, &current_segment, &current_packet_index)) {
        print_data("txrx", "dd", read_packet_num, 0);
        read_packet_num++;
        return true;
    }

    uint32_t recv_packet_num = current_segment.toInt();
    if (recv_packet_num != read_packet_num) {
        println_error("Received packet num doesn't match local count. %d != %d", recv_packet_num, read_packet_num);
        read_packet_num = recv_packet_num;
    }
    
    // find category segment
    if (get_segment(current_packet, &current_segment, &current_packet_index)) {
        print_data("txrx", "dd", read_packet_num, 0);
        read_packet_num++;
        return true;
    }
    String category = current_segment;

    // remove packet number and checksum
    current_packet = current_packet.substring(current_packet_index, current_packet.length() - 2);
    current_packet_index = 0;
    process_serial_packet(category, current_packet);

    print_data("txrx", "dd", read_packet_num, 1);

    read_packet_num++;
    return true;
}

void read_all_serial()
{
    if (!DATA_SERIAL.available()) {
        return;
    }
    size_t counter = 0;
    while (DATA_SERIAL.available()) {
        read_one_packet();
        counter++;
        if (counter > 0x10000) {
            break;
        }
    }
}

void print_data(String name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    String data = String(PACKET_START_0) + String(PACKET_START_1);
    data += String(write_packet_num) + "\t";
    data += name;
    while (*formats != '\0') {
        data += "\t";
        if (*formats == 'd') {
            int i = va_arg(args, int32_t);
            data += String(i);
        }
        else if (*formats == 'u') {
            uint32_t u = va_arg(args, uint32_t);
            data += String(u);
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            data += s;
        }
        else if (*formats == 'f') {
            double f = va_arg(args, double);
            data += String(f);
        }
        ++formats;
    }
    va_end(args);

    uint16_t calc_checksum = 0;
    unsigned int length = data.length();
    for (size_t index = 0; index < length; index++) {
        calc_checksum += (uint16_t)current_packet.charAt(index);
    }

    data += (calc_checksum >> 8) & 0xff;
    data += calc_checksum & 0xff;

    data += PACKET_STOP;
  
    // checksum might be inserting null characters. Force the buffer to extend
    // to include packet stop and checksum
    DATA_SERIAL.write(data.c_str(), length + 3);
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
