#ifndef ROVER6_SERIAL
#define ROVER6_SERIAL

#include <Arduino.h>

#include "rover6_general.h"

#define MSG_SERIAL  Serial
#define DATA_SERIAL  Serial5
#define SERIAL_MSG_BUFFER_SIZE 0xff
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
#define PACKET_END '\n'
const char PACKET_START_0 = 0x12;
const char PACKET_START_1 = 0x34;
bool is_reporting_enabled = false;
char* PACKET_BUFFER = new char[0xffff];
char* WRITE_BUFFER = new char[0xffff];
char* SUBFIELD_BUFFER = new char[0xff];
int32_t READ_PACKET_NUM = 0;
uint32_t WRITE_PACKET_NUM = 0;

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void print_info(const char* message, ...);
void println_info(const char* message, ...);
void print_error(const char* message, ...);
void println_error(const char* message, ...);

typedef struct subfield subfield;
struct subfield {
    float f;
    uint8_t s_len;
    char* s;
    int32_t i;
    int64_t l;
    char c;
};

subfield* FIELD_VAL;

union
{
    float f;
    uint8_t c[4];
} float_conv;
float parse_float(char* data) {
    float_conv.c[3] = data[0];
    float_conv.c[2] = data[1];
    float_conv.c[1] = data[2];
    float_conv.c[0] = data[3];

    return float_conv.f;
}
void float_to_str(char* buffer, float f) {
    float_conv.f = f;
    for (size_t index = 0; index < 4; index++) {
        buffer[index] = float_conv.c[index];
    }
}

union
{
    int32_t i;
    uint8_t c[4];
} int_conv;
int32_t parse_int(char* data) {
    int_conv.c[3] = data[0];
    int_conv.c[2] = data[1];
    int_conv.c[1] = data[2];
    int_conv.c[0] = data[3];

    return int_conv.i;
}

void int_to_str(char* buffer, int i) {
    int_conv.i = i;
    for (size_t index = 0; index < 4; index++) {
        buffer[index] = int_conv.c[index];
    }
}

union
{
    int64_t l;
    uint8_t c[8];
} long_conv;
int64_t parse_long(char* data) {
    for (byte i = 0; i < 8; i++) {
        long_conv.c[7 - i] = data[i];
    }
    return long_conv.l;
}
void long_to_str(char* buffer, int l) {
    long_conv.l = l;
    for (size_t index = 0; index < 8; index++) {
        buffer[index] = long_conv.c[index];
    }
}

void process_serial_subfield(uint32_t num, char packet_category, char subfield_type, subfield* field, uint16_t field_index);

void print_data(char category, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    WRITE_BUFFER[0] = PACKET_START_0;
    WRITE_BUFFER[1] = PACKET_START_1;
    uint16_t msg_start_index = 4;  // bits 2 and 3 are for total length, calculated later
    uint16_t index = msg_start_index;  // start writing to buffer just after the length bits

    // write the packet index number in the first 4 bits
    int_to_str(WRITE_BUFFER + msg_start_index, WRITE_PACKET_NUM);
    index += 4;

    while (*formats != '\0') {
        // subfield type
        WRITE_BUFFER[index++] = *formats;

        // write data
        switch (*formats)
        {
            case 'd':
            case 'i': int_to_str(WRITE_BUFFER + index, va_arg(args, int32_t)); index += 4; break;
            case 'l': long_to_str(WRITE_BUFFER + index, va_arg(args, int64_t)); index += 8; break;
            case 'f': float_to_str(WRITE_BUFFER + index, (float)va_arg(args, double)); index += 4; break;
            case 'c': WRITE_BUFFER[index] = (char)va_arg(args, int); index++; break;

            default: 
                println_error("Invalid format type: %c", *formats);
                return;
        }
        if (*formats == 's') {
            char *s = va_arg(args, char*);
            strcpy(WRITE_BUFFER + index, s);
            index += strlen(s);
        }
        ++formats;
    }
    va_end(args);

    // message length excludes the packet start and length itself
    // add in checksum length
    uint16_t msg_length = index - msg_start_index + 2;
    WRITE_BUFFER[2] = (char)(msg_length >> 8);
    WRITE_BUFFER[3] = (char)(msg_length & 0xff);

    // calculate checksum by summing index 3...checksum index - 2
    // Ignore integer overflow
    uint16_t checksum = 0;
    uint16_t msg_length_no_check = msg_length - 2;
    for (size_t check_index = msg_start_index; check_index < msg_length_no_check; check_index++) {
        checksum += WRITE_BUFFER[check_index + msg_start_index];
    }
    WRITE_BUFFER[index++] = (char)(checksum >> 8);
    WRITE_BUFFER[index++] = (char)(checksum & 0xff);
    WRITE_BUFFER[index++] = PACKET_END;
    // WRITE_BUFFER[index++] = '\0';

    DATA_SERIAL.write(WRITE_BUFFER, index);
    WRITE_PACKET_NUM++;
}

void report_read_success(bool success) {
    print_data(1, "lci", CURRENT_TIME, (char)success, READ_PACKET_NUM);
}

void read_all_serial()
{
    if (!DATA_SERIAL.available()) {
        return;
    }
    char length_c1 = '\0';
    char length_c2 = '\0';
    uint16_t msg_length = 0;
    uint16_t msg_length_no_checksum = 0;

    char packet_category = '\0';
    char subfield_type = '\0';
    uint8_t subfield_len = 0;

    char recv_checksum_c1 = '\0';
    char recv_checksum_c2 = '\0';
    uint16_t calc_checksum = 0;
    uint16_t recv_checksum = 0;

    uint16_t index = 0;
    uint16_t field_index = 0;

    while (DATA_SERIAL.available())
    {

        char c = DATA_SERIAL.read();
        if (c != PACKET_START_0) {
            continue;
        }
        while (!DATA_SERIAL.available());
        c = DATA_SERIAL.read();
        if (c != PACKET_START_1) {
            continue;
        }
        while (DATA_SERIAL.available() < 2);
        length_c1 = DATA_SERIAL.read();
        length_c2 = DATA_SERIAL.read();
        msg_length = (length_c1 << 8) | length_c2;
        if (msg_length < 5) {  // category, subfield, and one data byte + checksum bytes
            report_read_success(false);
            println_error("Invalid message length specified: %d", msg_length);
            return;
        }

        while (DATA_SERIAL.available() < msg_length + 1);
        DATA_SERIAL.readBytes(PACKET_BUFFER, msg_length + 1);
        msg_length_no_checksum = msg_length - 2;
        for (size_t check_index = 0; check_index < msg_length_no_checksum; check_index++) {
            calc_checksum += PACKET_BUFFER[check_index];
        }
        recv_checksum_c1 = PACKET_BUFFER[msg_length - 2];
        recv_checksum_c2 = PACKET_BUFFER[msg_length - 1];
        recv_checksum = (recv_checksum_c1 << 8) | recv_checksum_c2;

        if (recv_checksum != calc_checksum) {
            report_read_success(false);
            println_error("Received checksum %d does not equal calculated checksum: %d", recv_checksum, calc_checksum);
            return;
        }

        println_info("free memory: %d", freeMemory());
        DATA_SERIAL.write(PACKET_BUFFER, msg_length + 1);

        int32_t read_packet_num = parse_int(PACKET_BUFFER);
        if (READ_PACKET_NUM != read_packet_num) {
            println_error("Received packet number out of sync with local count: %d != %d", READ_PACKET_NUM, read_packet_num);
            READ_PACKET_NUM = read_packet_num;
        }
        println_info("Packet nums: local=%d, recv=%d", READ_PACKET_NUM, read_packet_num);
        
        index = 4;  // start index just after packet number
        // packet_category = PACKET_BUFFER[index];
        // println_info("Packet category: %c", packet_category);

        msg_length -= 2;  // don't parse the checksum, we did that already
        while (index < msg_length) {
            index++;
            subfield_type = PACKET_BUFFER[index];
            switch (subfield_type) {
                case 'f':
                case 'd':
                case 'i': subfield_len = 4; break;
                case 'l': subfield_len = 8; break;
                case 'c': subfield_len = 1; break;
                case 's': subfield_len = 0; break;
                case 0: 
                    report_read_success(false);
                    println_error("Null character encountered where subfield type expected at index %d", index);
                    return;
                default:
                    report_read_success(false);
                    println_error("Invalid subfield type: %c at index %d", subfield_type, index);
                    return;
            }
            index++;
            if (subfield_type != 's') {
                for (size_t sub_index = index; sub_index < index + subfield_len; index++) {
                    SUBFIELD_BUFFER[index] = PACKET_BUFFER[sub_index];
                }
            }

            switch (subfield_type) {
                case 'f': FIELD_VAL->f = parse_float(SUBFIELD_BUFFER); break;
                case 'd':
                case 'i': FIELD_VAL->i = parse_int(SUBFIELD_BUFFER); break;
                case 'l': FIELD_VAL->l = parse_long(SUBFIELD_BUFFER); break;
                case 'c': FIELD_VAL->c = SUBFIELD_BUFFER[0]; break;
                case 's':
                    strcpy(FIELD_VAL->s, PACKET_BUFFER + index);
                    subfield_len = strlen(FIELD_VAL->s);
                    break;
                case 0: 
                    report_read_success(false);
                    println_error("Null character encountered where subfield type expected at index %d", index);
                    return;
                default:
                    report_read_success(false);
                    println_error("Invalid subfield type: %c @ index %d", subfield_type, index);
                    return;
            }
            index += subfield_len;
            process_serial_subfield(READ_PACKET_NUM, packet_category, subfield_type, FIELD_VAL, field_index);
            field_index++;
        }
        report_read_success(true);
        READ_PACKET_NUM++;
    }
}

void print_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("INFO\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("INFO\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void print_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("ERROR\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("ERROR\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void setup_serial()
{
    // DATA_SERIAL.begin(115200);
    DATA_SERIAL.begin(500000);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
    println_info("Rover #6");
    println_info("Serial buses initialized.");

    subfield field;
    field.c = '\0';
    field.i = 0;
    field.l = 0;
    field.f = 0.0;
    field.s = new char[0xff];
    field.s_len = 0;
    FIELD_VAL = &field;
}

#endif // ROVER6_SERIAL
