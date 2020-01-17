#ifndef ROVER6_SERVOS
#define ROVER6_SERVOS

#include <Arduino.h>
#include <Adafruit_PWMServoDriverTeensy.h>

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

bool servos_on_standby = false;


void set_servo_standby(bool standby)
{
    if (servos_on_standby == standby) {
        return;
    }
    servos_on_standby = standby;
    if (standby) {  // set servos to low power
        servos.sleep();
    }
    else {  // bring servos out of standby mode
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

    set_servo_standby(false);
}

void set_servo(uint8_t n, int angle)
{
    if (servo_positions[n] != angle) {
        servo_positions[n] = angle;
        uint16_t pulse = (uint16_t)map(angle, 0, 180, servo_pulse_mins[n], servo_pulse_maxs[n]);
        servos.setPWM(n, 0, pulse);
    }
}

#endif // ROVER6_SERVOS
