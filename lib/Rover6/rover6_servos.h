#ifndef ROVER6_SERVOS
#define ROVER6_SERVOS

#include <Arduino.h>
#include <Adafruit_PWMServoDriverTeensy.h>

#include "rover6_general.h"
#include "rover6_serial.h"

/*
 * Adafruit PWM servo driver
 * PCA9685
 */

Adafruit_PWMServoDriver servos(0x40, &I2C_BUS_2);


#define NUM_SERVOS 16
#define SERVO_STBY 24

int servo_pulse_mins[NUM_SERVOS];
int servo_pulse_maxs[NUM_SERVOS];
int servo_positions[NUM_SERVOS];


void set_servos_active(bool active)
{
    if (safety_struct.are_servos_active == active) {
        return;
    }
    safety_struct.are_servos_active = active;
    if (active) {  // set servos to low power
        servos.sleep();
    }
    else {  // bring servos out of active mode
        servos.wakeup();
    }
}

void setup_servos()
{
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_mins[i] = 150;
    }
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_maxs[i] = 600;
    }

    servos.begin();
    servos.setPWMFreq(60);
    delay(10);
    println_info("PCA9685 Servos initialized.");
    pinMode(SERVO_STBY, OUTPUT);
    digitalWrite(SERVO_STBY, LOW);

    set_servos_active(false);
}

void set_servo(uint8_t n, int angle)
{
    if (servo_positions[n] != angle) {
        servo_positions[n] = angle;
        uint16_t pulse = (uint16_t)map(angle, 0, 180, servo_pulse_mins[n], servo_pulse_maxs[n]);
        servos.setPWM(n, 0, pulse);
    }
}

int get_servo(uint8_t n) {
    if (n >= NUM_SERVOS) {
        println_error("Requested servo num %d does not exist!", n);
        return -1;
    }
    return servo_positions[n];
}


#define FRONT_TILTER_SERVO_NUM 0
#define BACK_TILTER_SERVO_NUM 1
#define CAMERA_PAN_SERVO_NUM 2
#define CAMERA_TILT_SERVO_NUM 3

#define FRONT_TILTER_UP 70
#define FRONT_TILTER_DOWN 180
#define BACK_TILTER_UP 70
#define BACK_TILTER_DOWN 180


void set_front_tilter(int angle)
{
    if (angle < FRONT_TILTER_UP) {
        angle = FRONT_TILTER_UP;
    }
    if (angle > FRONT_TILTER_DOWN) {
        angle = FRONT_TILTER_DOWN;
    }
    set_servo(FRONT_TILTER_SERVO_NUM, angle);
}

void set_back_tilter(int angle)
{
    if (angle < BACK_TILTER_UP) {
        angle = BACK_TILTER_UP;
    }
    if (angle > BACK_TILTER_DOWN) {
        angle = BACK_TILTER_DOWN;
    }
    set_servo(BACK_TILTER_SERVO_NUM, angle);
}

#define CAMERA_PAN_UP 90
#define CAMERA_PAN_CENTER 43
#define CAMERA_PAN_DOWN 0
#define CAMERA_TILT_LEFT 0
#define CAMERA_TILT_CENTER 105
#define CAMERA_TILT_RIGHT 150

void center_camera()
{
    set_servo(CAMERA_PAN_SERVO_NUM, CAMERA_PAN_CENTER);
    set_servo(CAMERA_TILT_SERVO_NUM, CAMERA_TILT_CENTER);
}

void set_camera_pan(int angle)
{
    if (angle > CAMERA_PAN_DOWN) {
        angle = CAMERA_PAN_DOWN;
    }
    else if (angle < CAMERA_PAN_UP) {
        angle = CAMERA_PAN_UP;
    }
    set_servo(CAMERA_PAN_SERVO_NUM, angle);
}

void set_camera_tilt(int angle)
{
    if (angle > CAMERA_TILT_LEFT) {
        angle = CAMERA_TILT_LEFT;
    }
    else if (angle < CAMERA_TILT_RIGHT) {
        angle = CAMERA_TILT_RIGHT;
    }
    set_servo(CAMERA_TILT_SERVO_NUM, angle);
}

#endif // ROVER6_SERVOS
