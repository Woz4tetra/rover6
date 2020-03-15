
#ifndef ROVER6_I2C
#define ROVER6_I2C

#include <Arduino.h>
#include <i2c_t3.h>
#include "rover6_general.h"
#include "rover6_serial.h"

/*
 * I2C
 */

#define I2C_BUS_1 Wire
#define I2C_BUS_2 Wire1

namespace rover6_i2c
{
    void setup_i2c()
    {
        I2C_BUS_1.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
        I2C_BUS_1.setDefaultTimeout(200000); // 200ms
        I2C_BUS_2.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
        I2C_BUS_2.setDefaultTimeout(200000); // 200ms
        rover6_serial::println_info("I2C initialized.");
    }
};  // rover6_i2c
#endif // ROVER6_I2C
