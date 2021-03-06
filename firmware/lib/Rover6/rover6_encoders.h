
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

#define ENCODER_SAMPLERATE_DELAY_MS 33  // ~30 Hz

namespace rover6_encoders
{
    Encoder motorA_enc(MOTORA_ENCB, MOTORA_ENCA);
    Encoder motorB_enc(MOTORB_ENCA, MOTORB_ENCB);

    long encA_pos, encB_pos = 0;
    double enc_speedA, enc_speedB = 0.0;  // ticks/s, smoothed
    double enc_speedA_raw, enc_speedB_raw = 0.0;  // ticks/s

    uint32_t prev_enc_time = 0;

    double speed_smooth_kA = 1.0;
    double speed_smooth_kB = 1.0;

    // cm per rotation = 2pi * wheel radius (cm); arc = angle * radius
    // ticks per rotation = 1920
    // cm per tick = cm per rotation / ticks per rotation
    // double wheel_radius_cm = 32.5;
    // double cm_per_tick = 2.0 * PI * wheel_radius_cm / 1920.0; //  * ticks / rotation

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

        bool should_report = false;
        if (new_encA_pos != encA_pos || new_encB_pos != encB_pos) {
            should_report = true;
        }

        enc_speedA_raw = (double)(new_encA_pos - encA_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
        enc_speedB_raw = (double)(new_encB_pos - encB_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
        enc_speedA += speed_smooth_kA * (enc_speedA_raw - enc_speedA);
        enc_speedB += speed_smooth_kB * (enc_speedB_raw - enc_speedB);

        encA_pos = new_encA_pos;
        encB_pos = new_encB_pos;

        prev_enc_time = CURRENT_TIME;

        return should_report;
    }

    void report_encoders()
    {
        if (!rover6::rover_state.is_reporting_enabled) {
            return;
        }
        rover6_serial::data->write("enc", "uddff", CURRENT_TIME, encA_pos, encB_pos, enc_speedA, enc_speedB);
    }
}; // namespace rover6_encoders

#endif  // ROVER6_ENCODERS
