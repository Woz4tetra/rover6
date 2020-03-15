#ifndef ROVER6_BNO
#define ROVER6_BNO

#include <Arduino.h>
#include <Adafruit_BNO055_Teensy.h>

#include "rover6_i2c.h"
#include "rover6_serial.h"
#include "rover6_general.h"

/*
 * Adafruit 9-DOF Absolute Orientation IMU
 * BNO055
 */

#define BNO055_COPYSTRING(str, ...) strcpy(str, ##__VA_ARGS__)

#define BNO055_RST_PIN 25
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
#define BNO055_DATA_BUF_LEN 9

namespace rover6_bno
{
    sensors_event_t orientationData;
    sensors_event_t angVelocityData;
    sensors_event_t linearAccelData;
    int8_t bno_temperature = 0;

    int8_t bno_board_temp;
    Adafruit_BNO055 bno(-1, BNO055_ADDRESS_A, &I2C_BUS_2);
    bool is_bno_setup = false;
    bool is_bno_active = false;

    uint8_t bno_system_status = 0;
    uint8_t bno_self_test_result = 0;
    uint8_t bno_system_error = 0;
    char* bno_status_string = new char[0xff];


    uint32_t bno_report_timer = 0;
    #define BNO_SAMPLERATE_DELAY_MS 100

    void get_system_status_string(uint8_t system_status, char* status)
    {
        switch (system_status) {
            case 0: BNO055_COPYSTRING(status, "idle"); break;
            case 1: BNO055_COPYSTRING(status, "system error"); break;
            case 2: BNO055_COPYSTRING(status, "initializing peripherals"); break;
            case 3: BNO055_COPYSTRING(status, "system initialization"); break;
            case 4: BNO055_COPYSTRING(status, "executing self-test"); break;
            case 5: BNO055_COPYSTRING(status, "sensor fusion running"); break;
            case 6: BNO055_COPYSTRING(status, "sensor fusion not running"); break;
        }
    }

    void bno_print_system_status()
    {
        get_system_status_string(bno_system_status, bno_status_string);
        rover6_serial::println_info("BNO055 system status - %d, %s", bno_system_status, bno_status_string);
    }


    void bno_print_self_test()
    {
        if ((bno_self_test_result & 1) == 1) {
            BNO055_COPYSTRING(bno_status_string, "pass");
        }
        else {
            BNO055_COPYSTRING(bno_status_string, "fail");
        }
        rover6_serial::println_info("BNO055 accel self test - %s", bno_status_string);

        if (((bno_self_test_result >> 1) & 1) == 1) {
            BNO055_COPYSTRING(bno_status_string, "pass");
        }
        else {
            BNO055_COPYSTRING(bno_status_string, "fail");
        }
        rover6_serial::println_info("BNO055 mag self test - %s", bno_status_string);

        if (((bno_self_test_result >> 2) & 1) == 1) {
            BNO055_COPYSTRING(bno_status_string, "pass");
        }
        else {
            BNO055_COPYSTRING(bno_status_string, "fail");
        }
        rover6_serial::println_info("BNO055 gyro self test - %s", bno_status_string);

        if (((bno_self_test_result >> 3) & 1) == 1) {
            BNO055_COPYSTRING(bno_status_string, "pass");
        }
        else {
            BNO055_COPYSTRING(bno_status_string, "fail");
        }
        rover6_serial::println_info("BNO055 mcu self test - %s", bno_status_string);
    }

    void get_system_error_string(uint8_t system_error, char* status)
    {
        switch (system_error) {
            case 0: BNO055_COPYSTRING(status, "No error"); break;
            case 1: BNO055_COPYSTRING(status, "Peripheral initialization error"); break;
            case 2: BNO055_COPYSTRING(status, "System initialization error"); break;
            case 3: BNO055_COPYSTRING(status, "Self test result failed"); break;
            case 4: BNO055_COPYSTRING(status, "Register map value out of range"); break;
            case 5: BNO055_COPYSTRING(status, "Register map address out of range"); break;
            case 6: BNO055_COPYSTRING(status, "Register map write error"); break;
            case 7: BNO055_COPYSTRING(status, "BNO low power mode not available for selected operat ion mode"); break;
            case 8: BNO055_COPYSTRING(status, "Accelerometer power mode not available"); break;
            case 9: BNO055_COPYSTRING(status, "Fusion algorithm configuration error"); break;
            case 10: BNO055_COPYSTRING(status, "Sensor configuration error"); break;
        }
    }

    void bno_print_system_error()
    {
        get_system_error_string(bno_system_error, bno_status_string);
        rover6_serial::println_info("BNO055 system error - %d, %s", bno_system_error, bno_status_string);
    }

    void get_bno_status()
    {
        uint8_t new_bno_system_status;
        uint8_t new_bno_self_test_result;
        uint8_t new_bno_system_error;
        bno.getSystemStatus(&new_bno_system_status, &new_bno_self_test_result, &new_bno_system_error);
        if (new_bno_system_status != bno_system_status || new_bno_self_test_result != bno_self_test_result || new_bno_system_error != bno_system_error)
        {
            bno_system_status = new_bno_system_status;
            bno_self_test_result = new_bno_self_test_result;
            bno_system_error = new_bno_system_error;
            bno_print_system_status();
            bno_print_self_test();
            bno_print_system_error();
        }
    }

    void hardware_reset_bno()
    {
        digitalWrite(BNO055_RST_PIN, LOW);
        delay(100);
        digitalWrite(BNO055_RST_PIN, HIGH);
        delay(800);
        if (!bno.begin()) {
            rover6_serial::println_error("No BNO055 detected!! Check your wiring or I2C address");
            return;
        }
        bno.setExtCrystalUse(true);

        bno.getSystemStatus(&bno_system_status, &bno_self_test_result, &bno_system_error);
        bno_print_system_status();
        bno_print_self_test();
        bno_print_system_error();
    }


    void set_bno_active(bool active)
    {
        if (is_bno_active == active) {
            return;
        }
        is_bno_active = active;

        if (active) {
            // bno.enterNormalMode();
            hardware_reset_bno();
        }
        // else {
        //     bno.enterSuspendMode();
        // }
    }


    void setup_BNO055()
    {
        if (!bno.begin()) {
            rover6_serial::println_error("No BNO055 detected!! Check your wiring or I2C address");
            return;
        }
        pinMode(BNO055_RST_PIN, OUTPUT);
        digitalWrite(BNO055_RST_PIN, HIGH);

        delay(1000);
        is_bno_setup = true;
        rover6_serial::println_info("BNO055 initialized.");

        bno.setExtCrystalUse(true);
        delay(100);

        bno.getSystemStatus(&bno_system_status, &bno_self_test_result, &bno_system_error);
        bno_print_system_status();
        bno_print_self_test();
        bno_print_system_error();

        delay(100);

        set_bno_active(false);
    }

    bool read_BNO055()
    {
        if (!is_bno_setup || !is_bno_active) {
            return false;
        }

        if (CURRENT_TIME - bno_report_timer < BNO_SAMPLERATE_DELAY_MS) {
            return false;
        }
        bno_report_timer = CURRENT_TIME;

        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno_temperature = bno.getTemp();

        return true;
    }

    void report_BNO055()
    {
        if (!rover6::rover_state.is_reporting_enabled) {
            return;
        }

        rover6_serial::data->write(
            "bno", "ufffffffffd",
            CURRENT_TIME,
            orientationData.orientation.x,
            orientationData.orientation.y,
            orientationData.orientation.z,
            angVelocityData.gyro.x,
            angVelocityData.gyro.y,
            angVelocityData.gyro.z,
            linearAccelData.acceleration.x,
            linearAccelData.acceleration.y,
            linearAccelData.acceleration.z,
            bno_temperature
        );
    }
};  // namespace rover6_bno

#endif  // ROVER6_BNO
