
#ifndef ROVER6_ENCODERS
#define ROVER6_ENCODERS

#include <Arduino.h>
#include <Encoder.h>
#include "rover6_general.h"
#include "rover6_serial.h"


/*
 * Encoders
 */

#define MOTORA_ENCA 23
#define MOTORA_ENCB 22
#define MOTORB_ENCA 21
#define MOTORB_ENCB 20

Encoder motorA_enc(MOTORA_ENCA, MOTORA_ENCB);
Encoder motorB_enc(MOTORB_ENCB, MOTORB_ENCA);

#define ENCODER_SAMPLERATE_DELAY_MS 33  // ~30 Hz

long encA_pos, encB_pos = 0;
double enc_speedA, enc_speedB = 0.0;  // cm/s
uint32_t prev_enc_time = 0;

// cm per rotation = 2pi * wheel radius (cm); arc = angle * radius
// ticks per rotation = 1920
// cm per tick = cm per rotation / ticks per rotation
double wheel_radius_cm = 32.5;
double cm_per_tick = 2.0 * PI * wheel_radius_cm / 1920.0; //  * ticks / rotation

// 160 rpm @ 6V
// 135 rpm @ 5V
// 60 rpm @ 3V
// double max_rpm = 135.0;
// double max_linear_speed_cps = max_rpm * 2.0 * PI * wheel_radius_cm / 60.0;  // cm per s, no load
double max_linear_speed_cps = 915.0;
double cps_to_cmd = 255.0 / max_linear_speed_cps;

void reset_encoders()
{
    encA_pos = 0;
    encB_pos = 0;
    motorA_enc.write(0);
    motorB_enc.write(0);
}

bool read_encoders()
{
    if (CURRENT_TIME - prev_enc_time < ENCODER_SAMPLERATE_DELAY_MS) {
        return false;
    }
    
    long new_encA_pos = motorA_enc.read();
    long new_encB_pos = motorB_enc.read();

    enc_speedA = (new_encA_pos - encA_pos) * cm_per_tick / (CURRENT_TIME - prev_enc_time) * 1000.0;
    enc_speedB = (new_encB_pos - encB_pos) * cm_per_tick / (CURRENT_TIME - prev_enc_time) * 1000.0;

    encA_pos = new_encA_pos;
    encB_pos = new_encB_pos;

    prev_enc_time = CURRENT_TIME;

    return true;
}

void report_encoders()
{
    if (!rover_state.is_reporting_enabled) {
        return;
    }
    print_data("enc", "uddff", CURRENT_TIME, encA_pos, encB_pos, enc_speedA, enc_speedB);
}

#endif  // ROVER6_ENCODERS
