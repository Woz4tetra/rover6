#ifndef ROVER6_SERIAL
#define ROVER6_SERIAL

#include <Arduino.h>

#include "rover6_general.h"

#define MSG_SERIAL  Serial
#define DATA_SERIAL  Serial5
#define SERIAL_MSG_BUFFER_SIZE 0xff
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
//#define PACKET_END '\n'
#define PACKET_START '\n'
bool is_reporting_enabled = false;
char* PACKET_BUFFER = new char[0xffff];
char* WRITE_BUFFER = new char[0xffff];
char* SUBFIELD_BUFFER = new char[0xffff];
char* PACKET_NUM_BUFFER = new char[5];
uint32_t READ_PACKET_NUM = 0;
uint32_t WRITE_PACKET_NUM = 0;

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
    if (strlen(data) != 4) {
        println_error("Incorrect length for float parsing: %s", data);
        return 0.0;
    }
    float_conv.c[0] = data[0];
    float_conv.c[1] = data[1];
    float_conv.c[2] = data[2];
    float_conv.c[3] = data[3];

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
    char c[4];
} int_conv;
int32_t parse_int(char* data) {
    if (strlen(data) != 4) {
        println_error("Incorrect length for int parsing: %s", data);
        return 0;
    }
    int_conv.c[0] = data[0];
    int_conv.c[1] = data[1];
    int_conv.c[2] = data[2];
    int_conv.c[3] = data[3];

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
    if (strlen(data) != 8) {
        println_error("Incorrect length for int parsing: %s", data);
        return 0;
    }
    for (byte i = 0; i < 8; i++) {
        long_conv.c[i] = data[i];
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
    WRITE_BUFFER[0] = PACKET_START;
    uint16_t msg_start_index = 3;  // bits 1 and 2 are for total length, calculated later
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
    WRITE_BUFFER[1] = (char)(msg_length >> 8);
    WRITE_BUFFER[2] = (char)(msg_length & 0xff);

    // calculate checksum by summing index 3...checksum index - 2
    // Ignore integer overflow
    uint16_t checksum = 0;
    uint16_t msg_length_no_check = msg_length - 2;
    for (size_t check_index = msg_start_index; check_index < msg_length_no_check; check_index++) {
        checksum += WRITE_BUFFER[check_index + msg_start_index];
    }
    WRITE_BUFFER[index++] = (char)(checksum >> 8);
    WRITE_BUFFER[index++] = (char)(checksum & 0xff);
    WRITE_BUFFER[index] = '\0';

    DATA_SERIAL.write(WRITE_BUFFER);
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
    char length_c1;
    char length_c2;
    uint16_t msg_length = 0;
    uint16_t msg_length_no_checksum = 0;

    char packet_category;
    char subfield_type;
    uint8_t subfield_len;

    char recv_checksum_c1;
    char recv_checksum_c2;
    uint16_t calc_checksum = 0;
    uint16_t recv_checksum = 0;

    size_t index = 0;
    uint16_t field_index = 0;

    while (DATA_SERIAL.available())
    {
        if (DATA_SERIAL.read() != PACKET_START) {
            continue;
        }
        length_c1 = DATA_SERIAL.read();
        length_c2 = DATA_SERIAL.read();
        msg_length = (length_c1 << 8) | length_c2;
        if (msg_length < 5) {  // category, subfield, and one data byte + checksum bytes
            report_read_success(false);
            println_error("Invalid message length specified: %d", msg_length);
            return;
        }

        Serial.readBytes(PACKET_BUFFER, msg_length);
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

        index = 0;

        strncpy(PACKET_NUM_BUFFER, PACKET_BUFFER + index, 4);
        READ_PACKET_NUM = parse_int(PACKET_NUM_BUFFER);

        packet_category = PACKET_BUFFER[index];
        msg_length -= 2;  // don't parse the checksum, we did that already
        while (index < msg_length) {  
            subfield_type = PACKET_BUFFER[++index];
            switch (subfield_type) {
                case 'f':
                case 'i': subfield_len = 4; break;
                case 'l': subfield_len = 8; break;
                case 'c': subfield_len = 1; break;
                case 's':
                    FIELD_VAL->s_len = (uint8_t)PACKET_BUFFER[++index];
                    subfield_len = FIELD_VAL->s_len;
                    break;
                default:
                    report_read_success(false);
                    println_error("Invalid subfield type: %s", subfield_type);
                    return;
            }
            index++;
            strncpy(SUBFIELD_BUFFER, PACKET_BUFFER + index, subfield_len);
            SUBFIELD_BUFFER[subfield_len] = '\0';

            switch (subfield_type) {
                case 'f': FIELD_VAL->f = parse_float(SUBFIELD_BUFFER); break;
                case 'i': FIELD_VAL->i = parse_int(SUBFIELD_BUFFER); break;
                case 'l': FIELD_VAL->l = parse_long(SUBFIELD_BUFFER); break;
                case 'c': FIELD_VAL->c = SUBFIELD_BUFFER[0]; break;
                case 's':
                    strncpy(FIELD_VAL->s, PACKET_BUFFER + index, (uint8_t)FIELD_VAL->s_len);
                    FIELD_VAL->s[FIELD_VAL->s_len] = '\0';
                    break;
                default:
                    report_read_success(false);
                    println_error("Invalid subfield type: %s", subfield_type);
                    return;
            }
            index += subfield_len;
            process_serial_subfield(READ_PACKET_NUM, packet_category, subfield_type, FIELD_VAL, field_index);
            field_index++;
        }
        report_read_success(true);
    }
}

void print_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    MSG_SERIAL.print("INFO\t");
    MSG_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    MSG_SERIAL.print("INFO\t");
    MSG_SERIAL.print(SERIAL_MSG_BUFFER);
    MSG_SERIAL.print('\n');
}

void print_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    MSG_SERIAL.print("ERROR\t");
    MSG_SERIAL.print(SERIAL_MSG_BUFFER);
}

void println_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    MSG_SERIAL.print("ERROR\t");
    MSG_SERIAL.print(SERIAL_MSG_BUFFER);
    MSG_SERIAL.print('\n');
}

void setup_serial()
{
    MSG_SERIAL.begin(115200);
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
