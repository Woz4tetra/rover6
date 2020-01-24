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

#define BNO055_RST_PIN 25
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
#define BNO055_DATA_BUF_LEN 9

sensors_event_t orientationData;
sensors_event_t angVelocityData;
sensors_event_t linearAccelData;
int8_t bno_temperature = 0;

int8_t bno_board_temp;
Adafruit_BNO055 bno(-1, BNO055_ADDRESS_A, &I2C_BUS_2);
bool is_bno_setup = false;

uint32_t bno_report_timer = 0;
#define BNO_SAMPLERATE_DELAY_MS 100

void setup_BNO055()
{
    if (!bno.begin()) {
        println_error("No BNO055 detected!! Check your wiring or I2C address");
        return;
    }

    delay(1000);
    is_bno_setup = true;
    println_info("BNO055 initialized.");
 
    bno.setExtCrystalUse(true);
}

bool read_BNO055()
{
    if (!is_bno_setup) {
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
    if (!rover_state.is_reporting_enabled) {
        return;
    }

    print_data(
        "bno", "lfffffffffd",
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

#endif  // ROVER6_BNO
