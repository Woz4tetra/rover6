#include "rover6_general.h"
#include "rover6_tft.h"
#include "rover6_bno.h"
#include "rover6_encoders.h"
#include "rover6_fsr.h"
#include "rover6_ina.h"
#include "rover6_motors.h"
#include "rover6_servos.h"
#include "rover6_tof.h"



unsigned int ROW_SIZE = 10;
unsigned int BORDER_OFFSET_W = 3;
unsigned int BORDER_OFFSET_H = 1;

void init_menus()
{
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds("A", 0, 0, &x1, &y1, &w, &h);
    ROW_SIZE = h + BORDER_OFFSET_H;
}

void draw_sensor_data()
{
    tft.setCursor(0, 0);

    tft.print(String(ina219_current_mA));
    tft.print("mA, ");
    tft.print(String(ina219_loadvoltage));
    tft.println("V         ");

    tft.print("bno: ");
    tft.print(String(orientationData.orientation.x));
    tft.print(",");
    tft.print(String(orientationData.orientation.y));
    tft.print(",");
    tft.print(String(orientationData.orientation.z));
    tft.println("        ");

    tft.print("motor: ");
    tft.print(motorA.getSpeed());
    tft.print(",");
    tft.print(motorB.getSpeed());
    tft.print(",");
    tft.print(safety_struct.are_motors_active);
    tft.println("        ");

    tft.print("servo: ");
    tft.print(get_servo(FRONT_TILTER_SERVO_NUM));
    tft.print(",");
    tft.print(get_servo(BACK_TILTER_SERVO_NUM));
    tft.print(",");
    tft.print(get_servo(CAMERA_PAN_SERVO_NUM));
    tft.print(",");
    tft.print(get_servo(CAMERA_TILT_SERVO_NUM));
    tft.print(",");
    tft.print(safety_struct.are_motors_active);
    tft.println("        ");

    tft.print("enc: ");
    tft.print(encA_pos);
    tft.print(",");
    tft.print(encB_pos);
    tft.println("        ");

    tft.print("lox: ");
    tft.print(measure1.RangeMilliMeter);
    tft.print(",");
    tft.print(measure2.RangeMilliMeter);
    tft.print(",");
    tft.print(safety_struct.is_front_tof_ok);
    tft.print(",");
    tft.print(safety_struct.is_back_tof_ok);
    tft.print(",");
    tft.print(measure1.RangeStatus);
    tft.print(",");
    tft.print(measure2.RangeStatus);
    tft.println("        ");

    tft.print("fsr: ");
    tft.print(fsr_1_val);
    tft.print(",");
    tft.print(fsr_2_val);
    tft.println("        ");

    tft.print("safety: ");
    tft.print(is_safe_to_move());
    tft.print(",");
    tft.print(is_obstacle_in_front());
    tft.print(",");
    tft.print(is_obstacle_in_back());
    tft.println("        ");
}

const char* const MAIN_MENU_ENTRIES[] PROGMEM = {
    "Entry 1",
    "Entry 2",
    "Entry 3",
    "Entry 4",
    "Entry 5",
    "Entry 6",
    "Entry 7",
    "Entry 8",
    "Entry 9",
    "Entry 10",
    "Entry 11",
    "Entry 12"
};
#define MAIN_MENU_ENTRIES_LEN 12

bool MAIN_MENU_VISIBLE = false;
int MAIN_MENU_SELECT_INDEX = 0;
int PREV_MAIN_MENU_SELECT_INDEX = -1;
void draw_main_menu()
{
    MAIN_MENU_VISIBLE = true;
    if (MAIN_MENU_SELECT_INDEX < 0) {
        MAIN_MENU_SELECT_INDEX = 0;
    }
    if (MAIN_MENU_SELECT_INDEX >= MAIN_MENU_ENTRIES_LEN) {
        MAIN_MENU_SELECT_INDEX = MAIN_MENU_ENTRIES_LEN - 1;
    }
    
    // draw_sensor_data();

    // tft.fillScreen(ST7735_BLACK);
    if (PREV_MAIN_MENU_SELECT_INDEX >= 0)
    {
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * PREV_MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLACK
        );
    }
    
    for (size_t i = 0; i < MAIN_MENU_ENTRIES_LEN; i++)
    {
        tft.setCursor(BORDER_OFFSET_W, ROW_SIZE * i + BORDER_OFFSET_H);
        tft.print(MAIN_MENU_ENTRIES[i]);
    }

    tft.drawRect(
        BORDER_OFFSET_W - 1,
        ROW_SIZE * MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1,
        tft.width() - BORDER_OFFSET_W - 1,
        ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLUE
    );

    PREV_MAIN_MENU_SELECT_INDEX = MAIN_MENU_SELECT_INDEX;
}