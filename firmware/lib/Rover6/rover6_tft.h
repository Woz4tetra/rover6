
#ifndef ROVER6_TFT
#define ROVER6_TFT

#include <Arduino.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "rover6_general.h"
#include "rover6_serial.h"


/*
 * Adafruit TFT 1.8" display, 160x128
 * ST7735
 */

#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_LITE   6
#define TFT_UPDATE_DELAY_MS 250

namespace rover6_tft
{
    const float TFT_PI = 3.1415926;
    uint32_t tft_display_timer = 0;

    Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
    uint8_t tft_brightness;

    void set_display_brightness(int brightness)
    {
        analogWrite(TFT_LITE, brightness);
        tft_brightness = brightness;
    }

    void black_display() {
        tft.fillScreen(ST77XX_BLACK);
    }

    void initialize_display()
    {
        pinMode(TFT_LITE, OUTPUT);
        tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
        delay(10);
        set_display_brightness(255);
        black_display();
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

        tft.setTextWrap(false);
        tft.setTextSize(1);
        tft.setRotation(1); // horizontal display

        rover6_serial::println_info("TFT display initialized.");
        tft.print("Hello!\n");
    }
};

#endif  // ROVER6_TFT
