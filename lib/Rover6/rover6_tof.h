#ifndef ROVER6_TOF
#define ROVER6_TOF

#include <Arduino.h>
#include <Adafruit_VL53L0X_Teensy.h>
#include "rover6_i2c.h"
#include "rover6_serial.h"
#include "rover6_general.h"

/*
 * Adafruit TOF distance sensor
 * VL53L0X
 */

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// shutdown pins
#define SHT_LOX1 7
#define SHT_LOX2 5

Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

uint32_t lox_report_timer = 0;
#define LOX_SAMPLERATE_FAST_DELAY_MS 150
#define LOX_SAMPLERATE_SLOW_DELAY_MS 500
unsigned int lox_samplerate_delay_ms = LOX_SAMPLERATE_FAST_DELAY_MS;

int LOX_GROUND_UPPER_THRESHOLD_MM = 80;
int LOX_GROUND_LOWER_THRESHOLD_MM = 10;

int LOX_OBSTACLE_UPPER_THRESHOLD_MM = 100;
int LOX_OBSTACLE_LOWER_THRESHOLD_MM = 150;


void setup_VL53L0X()
{
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    println_info("Shutdown pins inited...");

    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    println_info("Both in reset mode...(pins are low)");
    delay(10);
    println_info("Starting...");

    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // initing LOX1
    if(!lox1.begin(LOX1_ADDRESS, false, &I2C_BUS_1)) {
        println_error("Failed to boot first VL53L0X");
        while(1);
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if(!lox2.begin(LOX2_ADDRESS, false, &I2C_BUS_1)) {
        println_error("Failed to boot second VL53L0X");
        while(1);
    }
    println_info("VL53L0X's initialized.");
}
void read_front_VL53L0X() {
    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
}

void read_back_VL53L0X() {
    lox2.rangingTest(&measure2, false);
}


void report_VL53L0X()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("lox", "ldddddd", millis(),
        measure1.RangeMilliMeter, measure2.RangeMilliMeter,
        measure1.RangeStatus, measure2.RangeStatus,
        lox1.Status, lox2.Status  // lookup table in vl53l0x_def.h line 133
    );
}

void is_front_ok_VL53L0X() {
    bool success = true;
    if (lox1.Status != VL53L0X_ERROR_NONE) {
        println_error("lox1 reported error %d", lox1.Status);
        success = false;
    }
    if (measure1.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure1.RangeStatus, status_string);
        println_error("lox1 measurement reported an error: %s", status_string);
        success = false;
    }
    return success;
}

void is_back_ok_VL53L0X() {
    bool success = true;
    if (lox2.Status != VL53L0X_ERROR_NONE) {
        println_error("lox2 reported error %d", lox2.Status);
        success = false;
    }
    if (measure2.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure2.RangeStatus, status_string);
        println_error("lox2 measurement reported an error: %s", status_string);
        success = false;
    }
    return success;
}

#endif  // ROVER6_TOF
