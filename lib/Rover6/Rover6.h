
#ifndef __ROVER6_H__
#define __ROVER6_H__

#include <Arduino.h>
#include <i2c_t3.h>
#include <Adafruit_INA219_Teensy.h>
#include <Adafruit_PWMServoDriverTeensy.h>
#include <Adafruit_VL53L0X_Teensy.h>

#include <TB6612.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include <Encoder.h>

#include <Adafruit_BNO055_Teensy.h>

#include <IRremote.h>

/*
 * Serial devices
 */

#define MSG_SERIAL Serial
#define DATA_SERIAL Serial5

/*
 * Adafruit PWM servo driver
 * PCA9685
 */

#define NUM_SERVOS 16

/*
 * Adafruit dual motor driver breakout + encoders
 * TB6612
 */
#define MOTOR_STBY 26
#define MOTORA_DR1 27
#define MOTORA_DR2 28
#define MOTORB_DR1 31
#define MOTORB_DR2 32
#define MOTORA_PWM 29
#define MOTORB_PWM 30

#define MOTORA_ENCA 23
#define MOTORA_ENCB 22
#define MOTORB_ENCA 21
#define MOTORB_ENCB 20


/*
 * Adafruit TOF distance sensor
 * VL53L0X
 */

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// shutdown pins
#define SHT_LOX1 7
#define SHT_LOX2 5

/*
 * Adafruit TFT 1.8" display
 * ST7735
 */
#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_LITE   6
const float TFT_PI = 3.1415926;

/*
 * Adafruit FSR
 * Interlink 402
 */
#define FSR_PIN_1 35
#define FSR_PIN_2 36

/*
 * Adafruit 9-DOF Absolute Orientation IMU
 * BNO055
 */
#define BNO055_RST_PIN 25
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
#define BNO055_DATA_BUF_LEN 9


const uint16_t FAST_SAMPLERATE_DELAY_MS = 10;

/*
 * Data formatting over UART serial
 */
#define PACKET_END '\n'


/*
 * IR remote receiver
 */

#define IR_RECEIVER_PIN 2


class Rover6 {
public:
    Rover6();
    static Rover6* self;

    void begin();

    void check_serial();
    void report_data();



private:
    bool is_idle;
    void set_idle(bool state);

    uint32_t current_time;
    uint32_t bno_report_timer;
    uint32_t fast_sensor_report_timer;

    void write(String name, const char *formats, ...);

    Adafruit_PWMServoDriver* servos;

    // pulse length count (out of 4096)
    int* servo_pulse_mins;
    int* servo_pulse_maxs;

    Adafruit_INA219* ina219;
    float ina219_shuntvoltage;
    float ina219_busvoltage;
    float ina219_current_mA;
    float ina219_loadvoltage;
    float ina219_power_mW;

    TB6612* motorA;
    TB6612* motorB;
    Encoder* motorA_enc;
    Encoder* motorB_enc;
    long encA_pos;
    long encB_pos;

    Adafruit_VL53L0X* lox1;
    Adafruit_VL53L0X* lox2;
    VL53L0X_RangingMeasurementData_t* measure1;
    VL53L0X_RangingMeasurementData_t* measure2;

    uint16_t fsr_1_val;
    uint16_t fsr_2_val;

    Adafruit_ST7735* tft;

    sensors_event_t* orientationData;
    sensors_event_t* angVelocityData;
    sensors_event_t* linearAccelData;
    int8_t bno_board_temp;
    Adafruit_BNO055* bno;

    IRrecv* irrecv;
    decode_results* irresults;
    bool ir_result_available;

    void report_status();
    void print_info(String s)  { MSG_SERIAL.print("INFO\t"); MSG_SERIAL.println(s); }
    void print_error(String s)  { MSG_SERIAL.print("ERROR\t"); MSG_SERIAL.println(s); }

    void setup_serial();
    void setup_i2c();

    void setup_servos();
    void set_servo(uint8_t n, double angle);

    void setup_INA219();
    void read_INA219();
    void report_INA219();

    void setup_motors();
    void set_motor_standby(bool standby);
    void setup_encoders();
    void set_motorA(int speed)  { motorA->setSpeed(speed); }
    void set_motorB(int speed)  { motorB->setSpeed(speed); }
    void read_encoders();
    void report_encoders();

    void setup_VL53L0X();
    void read_VL53L0X();
    void report_VL53L0X();

    void setup_fsrs();
    void read_fsrs();
    void report_fsrs();

    void initialize_display();
    void set_display_brightness(int brightness);

    void setup_BNO055();
    void read_BNO055();
    void report_BNO055();

    void setup_IR();
    void read_IR();
    void report_IR();
};

#endif // __ROVER6_H__
