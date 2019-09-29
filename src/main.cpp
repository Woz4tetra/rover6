#include <Arduino.h>
#include <Rover6.cpp>

Rover6* rover = new Rover6();
Rover6* Rover6::self = rover;


void setup() {
    rover->begin();
}

void loop() {
    rover->check_serial();
    rover->report_data();
    delayMicroseconds(BNO055_SAMPLERATE_DELAY_US);
}
