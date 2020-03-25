#ifndef ROVER6_SERVOS
#define ROVER6_SERVOS

#include <Arduino.h>
#include <Adafruit_PWMServoDriverTeensy.h>

#include "rover6_general.h"
#include "rover6_i2c.h"
#include "rover6_serial.h"

#define NUM_SERVOS 16
#define SERVO_STBY 24

/*
 * Adafruit PWM servo driver
 * PCA9685
 */

namespace rover6_servos
{
    Adafruit_PWMServoDriver servos(0x40, &I2C_BUS_2);

    int servo_pulse_mins[NUM_SERVOS];
    int servo_pulse_maxs[NUM_SERVOS];
    int servo_positions[NUM_SERVOS];
    int servo_max_positions[NUM_SERVOS];
    int servo_min_positions[NUM_SERVOS];
    int servo_default_positions[NUM_SERVOS];

    double servo_cmd_to_angle_m = 0.0;


    #define FRONT_TILTER_SERVO_NUM 0
    #define BACK_TILTER_SERVO_NUM 1

    #define CAMERA_PAN_SERVO_NUM 2
    #define CAMERA_TILT_SERVO_NUM 3

    #define FRONT_TILTER_UP 90
    #define FRONT_TILTER_DOWN 0
    #define BACK_TILTER_UP 90
    #define BACK_TILTER_DOWN 0

    #define CAMERA_PAN_RIGHT 90
    #define CAMERA_PAN_CENTER 43
    #define CAMERA_PAN_LEFT 0
    #define CAMERA_TILT_UP 0
    #define CAMERA_TILT_CENTER 105
    #define CAMERA_TILT_DOWN 150


    #define FRONT_TILTER_DEFAULT FRONT_TILTER_UP
    #define BACK_TILTER_DEFAULT BACK_TILTER_UP
    #define CAMERA_PAN_DEFAULT CAMERA_PAN_CENTER
    #define CAMERA_TILT_DEFAULT CAMERA_TILT_CENTER

    void set_servos_active(bool active);
    void set_servo(uint8_t n, int angle);
    void set_servos_default();

    void setup_servos()
    {
        for (size_t i = 0; i < NUM_SERVOS; i++) {
            servo_pulse_mins[i] = 150;
            servo_pulse_maxs[i] = 600;
            servo_positions[i] = 0;
            servo_max_positions[i] = 0;
            servo_min_positions[i] = 0;
            servo_default_positions[i] = 0;
        }

        servos.begin();
        servos.setPWMFreq(60);
        delay(10);
        pinMode(SERVO_STBY, OUTPUT);
        digitalWrite(SERVO_STBY, LOW);

        servo_max_positions[FRONT_TILTER_SERVO_NUM] = FRONT_TILTER_UP;
        servo_max_positions[BACK_TILTER_SERVO_NUM] = BACK_TILTER_UP;
        servo_max_positions[CAMERA_PAN_SERVO_NUM] = CAMERA_PAN_RIGHT;
        servo_max_positions[CAMERA_TILT_SERVO_NUM] = CAMERA_TILT_DOWN;

        servo_min_positions[FRONT_TILTER_SERVO_NUM] = FRONT_TILTER_DOWN;
        servo_min_positions[BACK_TILTER_SERVO_NUM] = BACK_TILTER_DOWN;
        servo_min_positions[CAMERA_PAN_SERVO_NUM] = CAMERA_PAN_LEFT;
        servo_min_positions[CAMERA_TILT_SERVO_NUM] = CAMERA_TILT_UP;

        servo_default_positions[FRONT_TILTER_SERVO_NUM] = FRONT_TILTER_UP;
        servo_default_positions[BACK_TILTER_SERVO_NUM] = BACK_TILTER_UP;
        servo_default_positions[CAMERA_PAN_SERVO_NUM] = CAMERA_PAN_CENTER;
        servo_default_positions[CAMERA_TILT_SERVO_NUM] = CAMERA_TILT_CENTER;

        rover6::safety_struct.are_servos_active = false;
        // servos.sleep();
        set_servos_default();
        set_servos_active(true);

        servo_cmd_to_angle_m = (360.0 - 270.0) / ((double)BACK_TILTER_UP - (double)BACK_TILTER_DOWN);
        rover6_serial::println_info("PCA9685 Servos initialized.");
    }


    void set_servos_default()
    {
        // rover6_serial::println_info("set_servos_default");
        for (size_t i = 0; i < NUM_SERVOS; i++) {
            set_servo(i, servo_default_positions[i]);
        }
    }

    void set_servos_current()
    {
        // rover6_serial::println_info("set_servos_current");
        for (size_t i = 0; i < NUM_SERVOS; i++) {
            set_servo(i, servo_default_positions[i]);
        }
    }


    void set_servos_active(bool active)
    {
        if (rover6::safety_struct.are_servos_active == active) {
            return;
        }
        rover6::safety_struct.are_servos_active = active;
        if (active) {  // bring servos out of active mode
            servos.wakeup();
            set_servos_current();
        }
        else {  // set servos to low power
            servos.sleep();
        }
    }


    double tilter_servo_cmd_to_angle(int command) {
        return servo_cmd_to_angle_m * ((double)command - 270.0) + BACK_TILTER_DOWN;
    }


    void set_servo(uint8_t n, int angle)
    {
        if (!(0 <= n && n < NUM_SERVOS)) {
            return;
        }
        if (angle < servo_min_positions[n]) {
            angle = servo_min_positions[n];
        }
        if (angle > servo_max_positions[n]) {
            angle = servo_max_positions[n];
        }

        if (servo_positions[n] != angle) {
            servo_positions[n] = angle;
            uint16_t pulse = (uint16_t)map(angle, 0, 180, servo_pulse_mins[n], servo_pulse_maxs[n]);
            // rover6_serial::println_info("Servo %d: %ddeg, %d", n, angle, pulse);
            servos.setPWM(n, 0, pulse);
        }
    }

    void set_servo(uint8_t n) {
        set_servo(n, servo_default_positions[n]);
    }

    int get_servo(uint8_t n) {
        if (n >= NUM_SERVOS) {
            rover6_serial::println_error("Requested servo num %d does not exist!", n);
            return -1;
        }
        return servo_positions[n];
    }

    void report_servo_pos() {
        rover6_serial::data->write("servo", "udddddddddddddddd", CURRENT_TIME,
            servo_positions[0], servo_positions[1], servo_positions[2], servo_positions[3],
            servo_positions[4], servo_positions[5], servo_positions[6], servo_positions[7],
            servo_positions[8], servo_positions[9], servo_positions[10], servo_positions[11],
            servo_positions[12], servo_positions[13], servo_positions[14], servo_positions[15]
        );
    }

    void set_front_tilter(int angle) {
        set_servo(FRONT_TILTER_SERVO_NUM, angle);
    }

    void set_back_tilter(int angle) {
        set_servo(BACK_TILTER_SERVO_NUM, angle);
    }

    void center_camera() {
        set_servo(CAMERA_PAN_SERVO_NUM, CAMERA_PAN_CENTER);
        set_servo(CAMERA_TILT_SERVO_NUM, CAMERA_TILT_CENTER);
    }

    void set_camera_pan(int angle) {
        set_servo(CAMERA_PAN_SERVO_NUM, angle);
    }

    void set_camera_tilt(int angle) {
        set_servo(CAMERA_TILT_SERVO_NUM, angle);
    }

};  // namespace rover6_servos

#endif // ROVER6_SERVOS
