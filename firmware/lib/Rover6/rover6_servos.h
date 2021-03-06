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
    float servo_velocities[NUM_SERVOS];  // ticks per loop

    double servo_cmd_to_angle_m = 0.0;

    #define SERVO_UPDATE_DELAY_MS 100
    const float vel_duty_time_period = 1.0;
    const float servo_update_delay_s = 1E-3 * SERVO_UPDATE_DELAY_MS;
    uint32_t prev_servo_time = 0;

    #define FRONT_TILTER_SERVO_NUM 0
    #define BACK_TILTER_SERVO_NUM 1

    #define CAMERA_PAN_SERVO_NUM 2
    #define CAMERA_TILT_SERVO_NUM 3

    #define FRONT_TILTER_UP 90
    #define FRONT_TILTER_DOWN 0
    #define BACK_TILTER_UP 90
    #define BACK_TILTER_DOWN 0

    #define CAMERA_PAN_RIGHT 90
    #define CAMERA_PAN_CENTER 45
    #define CAMERA_PAN_LEFT 0
    #define CAMERA_TILT_UP 0
    #define CAMERA_TILT_CENTER 100
    #define CAMERA_TILT_DOWN 150


    #define FRONT_TILTER_DEFAULT FRONT_TILTER_UP
    #define BACK_TILTER_DEFAULT BACK_TILTER_UP
    #define CAMERA_PAN_DEFAULT CAMERA_PAN_CENTER
    #define CAMERA_TILT_DEFAULT CAMERA_TILT_CENTER

    void set_servos_active(bool active);
    void set_servo(uint8_t n, int angle);
    void set_servos_default();
    void report_servo_pos(uint8_t n);

    void setup_servos()
    {
        for (size_t i = 0; i < NUM_SERVOS; i++) {
            servo_pulse_mins[i] = 150;
            servo_pulse_maxs[i] = 600;
            servo_positions[i] = 0;
            servo_max_positions[i] = 0;
            servo_min_positions[i] = 0;
            servo_default_positions[i] = 0;
            servo_velocities[i] = 0.0;
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

    bool _set_servo(uint8_t n, int angle)
    {
        if (!(0 <= n && n < NUM_SERVOS)) {
            return false;
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
            report_servo_pos(n);
        }

        return true;
    }

    void set_servo(uint8_t n, int angle)
    {
        if (_set_servo(n, angle)) {
            servo_velocities[n] = 0.0;  // clear velocity commands
        }
    }

    void set_servo(uint8_t n) {
        set_servo(n, servo_default_positions[n]);
    }

    void set_velocity(uint8_t n, float command) {
        // command is in servo ticks per second
        servo_velocities[n] = servo_update_delay_s * command;  // convert command to servo ticks per loop
    }

    int get_servo(uint8_t n) {
        if (n >= NUM_SERVOS) {
            rover6_serial::println_error("Requested servo num %d does not exist!", n);
            return -1;
        }
        return servo_positions[n];
    }

    void report_servo_pos(uint8_t n) {
        rover6_serial::data->write("servo", "udd", CURRENT_TIME, n, servo_positions[n]);
    }

    void update()
    {
        if (CURRENT_TIME - prev_servo_time < SERVO_UPDATE_DELAY_MS) {
            return;
        }

        for (size_t n = 0; n < NUM_SERVOS; n++)
        {
            if (servo_velocities[n] == 0.0) {
                continue;
            }

            double int_part = 0.0;
            float frac_part = (float)modf(abs(servo_velocities[n]), &int_part);
            int vel_command = (int)(int_part);

            float mod_cycle = (float)fmod(1E-3 * CURRENT_TIME, vel_duty_time_period);
            if (mod_cycle < frac_part) {
                vel_command++;
            }
            // rover6_serial::println_info("vel_command, %d: %d", n, vel_command);
            vel_command = (int)(copysign(vel_command, servo_velocities[n]) + servo_positions[n]);

            _set_servo(n, vel_command);
        }
        prev_servo_time = CURRENT_TIME;
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
