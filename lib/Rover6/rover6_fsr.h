
#ifndef ROVER6_FSR
#define ROVER6_FSR

/*
 * Adafruit FSR
 * Interlink 402
 */

#include <Arduino.h>
#include "rover6_serial.h"
#include "rover6_general.h"

#define FSR_PIN_1 35
#define FSR_PIN_2 36

uint16_t fsr_1_val;
uint16_t fsr_2_val;
uint32_t fsr_report_timer = 0;

#define FSR_SAMPLERATE_DELAY_MS 250

#define FSR_CONTACT_THRESHOLD 50
#define FSR_NOISE_THRESHOLD 2

void setup_fsrs()
{
    pinMode(FSR_PIN_1, INPUT);
    pinMode(FSR_PIN_2, INPUT);
    println_info("FSRs initialized.");
}


bool is_left_bumper_in_contact() {
    return fsr_1_val >= FSR_CONTACT_THRESHOLD;
}

bool is_right_bumper_in_contact() {
    return fsr_2_val >= FSR_CONTACT_THRESHOLD;
}

bool read_fsrs()
{
    if (CURRENT_TIME - fsr_report_timer < FSR_SAMPLERATE_DELAY_MS) {
        return false;
    }
    fsr_1_val = analogRead(FSR_PIN_1);
    fsr_2_val = analogRead(FSR_PIN_2);

    safety_struct.is_left_bumper_trig = is_left_bumper_in_contact();
    safety_struct.is_right_bumper_trig = is_right_bumper_in_contact();
    return true;
}

void report_fsrs()
{
    if (!rover_state.is_reporting_enabled) {
        return;
    }
    print_data("fsr", "ldd", millis(), fsr_1_val, fsr_2_val);
}

#endif  // ROVER6_FSR
