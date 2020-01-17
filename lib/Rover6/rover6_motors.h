#ifndef ROVER6_MOTORS
#define ROVER6_MOTORS


#include <Arduino.h>
#include <TB6612.h>

#include "rover6_general.h"

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
uint32_t prev_commandA_time = 0;
uint32_t prev_commandB_time = 0;

bool motors_on_standby = false;
bool is_safe_to_move = false;
bool obstacle_in_front = false;
bool obstacle_in_back = false;


TB6612 motorA(MOTORA_PWM, MOTORA_DR2, MOTORA_DR1);
TB6612 motorB(MOTORB_PWM, MOTORB_DR1, MOTORB_DR2);

void set_motor_standby(bool standby)
{
    motors_on_standby = standby;
    if (standby) {  // set motors to low power
        digitalWrite(MOTOR_STBY, LOW);
    }
    else {  // bring motors out of standby mode
        digitalWrite(MOTOR_STBY, HIGH);
    }
}

void disable_motors() {
    safety_is_calibrated = false;
    is_safe_to_move = false;
}

void enable_motors() {
    safety_is_calibrated = true;
    is_safe_to_move = true;
}

void setup_motors()
{
    pinMode(MOTOR_STBY, OUTPUT);
    motorA.begin();
    motorB.begin();
    println_info("Motors initialized.");
    set_motor_standby(true);
}

void set_motorA(int speed) {
    if (is_safe_to_move || speed == 0) {
        motorA.setSpeed(speed);
    }
}
void set_motorB(int speed) {
    if (is_safe_to_move || speed == 0) {
        motorB.setSpeed(speed);
    }
}

bool is_moving() {
    return motorA.getSpeed() != 0 || motorB.getSpeed() != 0;
}
bool is_moving_forward() {
    return (motorA.getSpeed() + motorB.getSpeed()) >> 1 >= 0;
}

void stop_motors() {
    motorA.setSpeed(0);
    motorB.setSpeed(0);
}


void set_motors(int speedA, int speedB)
{
    if (obstacle_in_front && is_moving_forward()) {  // if an obstacle is detected in the front, only allow backwards commands
        stop_motors();
        return;
    }
    if (obstacle_in_back && !is_moving_forward()) {  // if an obstacle is detected in the back, only allow forwards commands
        stop_motors();
        return;
    }
    set_motorA(speedA);
    set_motorB(speedB);
}

void reset_motor_timeouts()
{
    prev_commandA_time = CURRENT_TIME;
    prev_commandB_time = CURRENT_TIME;
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
    return timedout;
}

#endif  // ROVER6_MOTORS
