#ifndef ROVER6_MOTORS
#define ROVER6_MOTORS


#include <Arduino.h>
#include <TB6612.h>

#include "rover6_general.h"
#include "rover6_serial.h"

/*
 * Adafruit dual motor driver breakout
 * TB6612
 */

#define MOTOR_STBY 26
#define MOTORA_DR1 27
#define MOTORA_DR2 28
#define MOTORB_DR1 31
#define MOTORB_DR2 32
#define MOTORA_PWM 29
#define MOTORB_PWM 30

#define MOTOR_COMMAND_TIMEOUT_MS 1000

namespace rover6_motors
{
    uint32_t prev_commandA_time = 0;
    uint32_t prev_commandB_time = 0;


    TB6612 motorA(MOTORA_PWM, MOTORA_DR1, MOTORA_DR2);
    TB6612 motorB(MOTORB_PWM, MOTORB_DR2, MOTORB_DR1);

    void set_motors_active(bool active)
    {
        if (rover6::safety_struct.are_motors_active == active) {
            return;
        }
        rover6::safety_struct.are_motors_active = active;
        if (active) {  // bring motors out of standby mode
            digitalWrite(MOTOR_STBY, HIGH);
        }
        else {  // set motors to low power
            digitalWrite(MOTOR_STBY, LOW);
        }
    }


    void setup_motors()
    {
        pinMode(MOTOR_STBY, OUTPUT);
        motorA.begin();
        motorB.begin();
        rover6_serial::println_info("Motors initialized.");
        set_motors_active(false);
    }

    void reset_motor_timeouts()
    {
        prev_commandA_time = CURRENT_TIME;
        prev_commandB_time = CURRENT_TIME;
    }

    void set_motorA(int speed) {
        if (rover6::is_safe_to_move() || speed == 0) {
            reset_motor_timeouts();
            motorA.setSpeed(speed);
        }
    }
    void set_motorB(int speed) {
        if (rover6::is_safe_to_move() || speed == 0) {
            reset_motor_timeouts();
            motorB.setSpeed(speed);
        }
    }

    bool is_moving() {
        return motorA.getSpeed() != 0 || motorB.getSpeed() != 0;
    }
    bool is_moving(int speedA, int speedB) {  // check a command that's about to send
        return speedA != 0 || speedB != 0;
    }
    bool is_moving_forward() {
        return motorA.getSpeed() + motorB.getSpeed() >= 0;
    }
    bool is_moving_forward(int speedA, int speedB) {
        return (speedA + speedB) >> 1 >= 0;
    }

    void stop_motors() {
        motorA.setSpeed(0);
        motorB.setSpeed(0);
    }


    void set_motors(int speedA, int speedB)
    {
        if (rover6::is_obstacle_in_front() && is_moving_forward(speedA, speedB)) {  // if an obstacle is detected in the front, only allow backwards commands
            speedA = 0;
            speedB = 0;
        }
        if (rover6::is_obstacle_in_back() && !is_moving_forward(speedA, speedB)) {  // if an obstacle is detected in the back, only allow forwards commands
            speedA = 0;
            speedB = 0;
        }
        set_motorA(speedA);
        set_motorB(speedB);
    }


    bool check_motor_timeout()
    {
        bool timedout = false;
        if (CURRENT_TIME - prev_commandA_time > MOTOR_COMMAND_TIMEOUT_MS) {
            motorA.setSpeed(0);
            timedout = true;
        }
        if (CURRENT_TIME - prev_commandB_time > MOTOR_COMMAND_TIMEOUT_MS) {
            motorB.setSpeed(0);
            timedout = true;
        }

        if (rover6::is_obstacle_in_front() && is_moving_forward()) {  // if an obstacle is detected in the front, only allow backwards commands
            stop_motors();
        }
        if (rover6::is_obstacle_in_back() && !is_moving_forward()) {  // if an obstacle is detected in the back, only allow forwards commands
            stop_motors();
        }

        return timedout;
    }
};  // namespace rover6_motors

#endif  // ROVER6_MOTORS
