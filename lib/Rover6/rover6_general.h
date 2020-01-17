
#ifndef ROVER6_GENERAL
#define ROVER6_GENERAL

#include <Arduino.h>

#include "rover6_serial.h"

// uint32_t current_time = 0;
#define CURRENT_TIME millis()


/*
 * Soft restart
 */
//
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

void soft_restart()
{
    DATA_SERIAL.end();  // clears the serial monitor  if used
    SCB_AIRCR = 0x05FA0004;  // write value for restart
}

// Safety systems
bool safety_is_calibrated = false;


#endif // ROVER6_GENERAL
