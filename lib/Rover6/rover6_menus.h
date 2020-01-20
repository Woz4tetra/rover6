#ifndef ROVER6_MENUS
#define ROVER6_MENUS


#include "rover6_general.h"
#include "rover6_serial.h"
#include "rover6_tft.h"
#include "rover6_bno.h"
#include "rover6_encoders.h"
#include "rover6_fsr.h"
#include "rover6_ina.h"
#include "rover6_motors.h"
#include "rover6_servos.h"
#include "rover6_tof.h"
#include "rover6_pid.h"


unsigned int ROW_SIZE = 10;
unsigned int BORDER_OFFSET_W = 3;
unsigned int BORDER_OFFSET_H = 1;

unsigned int TOP_BAR_H = 20;


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


// 
// Top bar
// 

uint8_t topbar_rpi_icon_x = 10;
uint8_t topbar_rpi_icon_y = TOP_BAR_H / 2;
uint8_t topbar_rpi_icon_r = (TOP_BAR_H - 4) / 2;
void draw_rpi_icon()
{
    if (rover_state.is_reporting_enabled) {
        tft.fillCircle(topbar_rpi_icon_x, topbar_rpi_icon_y, topbar_rpi_icon_r, ST77XX_GREEN);
    }
    else {
        tft.fillCircle(topbar_rpi_icon_x, topbar_rpi_icon_y, topbar_rpi_icon_r, ST77XX_RED);
    }
}

uint8_t topbar_active_icon_x = 30;
uint8_t topbar_active_icon_y = topbar_rpi_icon_y;
uint8_t topbar_active_icon_r = topbar_rpi_icon_r;
void draw_active_icon()
{
    if (rover_state.is_active) {
        tft.fillCircle(topbar_active_icon_x, topbar_active_icon_y, topbar_active_icon_r, ST77XX_GREEN);
    }
    else {
        tft.fillCircle(topbar_active_icon_x, topbar_active_icon_y, topbar_active_icon_r, ST77XX_RED);
    }
}

void draw_datestr()
{
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds(rpi_date_str, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(tft.width() / 2 - w / 2, TOP_BAR_H / 2 - h / 2);
    if (CURRENT_TIME - prev_date_str_update > 1000) {
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        if (CURRENT_TIME - prev_date_str_update > 5000) {
            tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        }
    }

    tft.print(rpi_date_str);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}

String battery_mA_menu_str;
String battery_V_menu_str;
void draw_battery()
{
    battery_mA_menu_str = "  " + String((int)ina219_current_mA) + "mA";
    battery_V_menu_str = "  " + String(ina219_loadvoltage) + "V";
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds(battery_mA_menu_str, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(tft.width() - w - 2, TOP_BAR_H / 2 - h);
    tft.print(battery_mA_menu_str);

    tft.getTextBounds(battery_V_menu_str, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(tft.width() - w - 2, TOP_BAR_H / 2);
    tft.print(battery_V_menu_str);
}

void draw_topbar()
{
    draw_battery();
    draw_rpi_icon();
    draw_active_icon();
    draw_datestr();
}

// 
// Notifications
// 



// 
// Main menu
// 

enum menu_names {
    MAIN_MENU,
    IMU_MENU,
    MOTORS_MENU,
    SAFETY_MENU,
    HOTSPOT_MENU,
    WIFI_MENU,
    CAMERA_MENU,
    LIDAR_MENU,
    SHUTDOWN_MENU,
    NONE_MENU,
    NOTIFICATION_MENU
};

const menu_names MAIN_MENU_ENUM_MAPPING[] PROGMEM = {
    IMU_MENU,
    MOTORS_MENU,
    SAFETY_MENU,
    HOTSPOT_MENU,
    WIFI_MENU,
    CAMERA_MENU,
    LIDAR_MENU,
    SHUTDOWN_MENU
    // "Entry 10",
    // "Entry 11",
    // "Entry 12"
};

const char* const MAIN_MENU_ENTRIES[] PROGMEM = {
    "IMU",
    "Motors",
    "Safety Systems",
    "Hotspot",
    "Show Wifi Settings",
    "Camera",
    "LIDAR",
    "Shutdown/restart"
    // "Entry 10",
    // "Entry 11",
    // "Entry 12"
};
const int MAIN_MENU_ENTRIES_LEN = 8;

menu_names DISPLAYED_MENU = MAIN_MENU;
menu_names PREV_DISPLAYED_MENU = NONE_MENU;  // for detecting screen change events

int MAIN_MENU_SELECT_INDEX = 0;
int PREV_MAIN_MENU_SELECT_INDEX = -1;
void draw_main_menu()
{
    if (MAIN_MENU_SELECT_INDEX < 0) {
        MAIN_MENU_SELECT_INDEX = 0;
    }
    if (MAIN_MENU_SELECT_INDEX >= MAIN_MENU_ENTRIES_LEN) {
        MAIN_MENU_SELECT_INDEX = MAIN_MENU_ENTRIES_LEN - 1;
    }

    if (PREV_MAIN_MENU_SELECT_INDEX == MAIN_MENU_SELECT_INDEX) {
        return;
    }
    
    // draw_sensor_data();

    // tft.fillScreen(ST7735_BLACK);
    if (PREV_MAIN_MENU_SELECT_INDEX >= 0)
    {
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * PREV_MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1 + TOP_BAR_H,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLACK
        );
    }
    
    for (int i = 0; i < MAIN_MENU_ENTRIES_LEN; i++)
    {
        tft.setCursor(BORDER_OFFSET_W, ROW_SIZE * i + BORDER_OFFSET_H + TOP_BAR_H);
        tft.print(MAIN_MENU_ENTRIES[i]);
    }

    tft.drawRect(
        BORDER_OFFSET_W - 1,
        ROW_SIZE * MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1 + TOP_BAR_H,
        tft.width() - BORDER_OFFSET_W - 1,
        ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLUE
    );

    PREV_MAIN_MENU_SELECT_INDEX = MAIN_MENU_SELECT_INDEX;
}

// 
// IMU menu
// 
const double IMU_DRAW_COMPASS_RADIUS = 30;
int16_t imu_draw_x0 = 0;
int16_t imu_draw_y0 = 0;
int16_t imu_draw_x1 = 0;
int16_t imu_draw_y1 = 0;
int16_t imu_draw_center_x = 0;
int16_t imu_draw_center_y = 0;
double imu_draw_prev_angle = 0.0;
void draw_imu_menu()
{
    tft.setCursor(BORDER_OFFSET_W, TOP_BAR_H + 5); tft.println("X: " + String(orientationData.orientation.x) + "   ");
    tft.setCursor(BORDER_OFFSET_W, ROW_SIZE + TOP_BAR_H + 5); tft.println("Y: " + String(orientationData.orientation.y) + "   ");
    tft.setCursor(BORDER_OFFSET_W, ROW_SIZE * 2 + TOP_BAR_H + 5); tft.println("Z: " + String(orientationData.orientation.z) + "   ");

    if (imu_draw_prev_angle == orientationData.orientation.x) {
        return;
    }
    imu_draw_prev_angle = orientationData.orientation.x;

    tft.drawLine(imu_draw_x0, imu_draw_y0, imu_draw_x1, imu_draw_y1, ST7735_BLACK);
    tft.drawCircle(imu_draw_x1, imu_draw_y1, 5, ST7735_BLACK);

    double angle_rad = 2 * PI - orientationData.orientation.x * PI / 180.0 - PI / 2;
    double x = IMU_DRAW_COMPASS_RADIUS * cos(angle_rad) / 2;
    double y = IMU_DRAW_COMPASS_RADIUS * sin(angle_rad) / 2;

    imu_draw_center_x = tft.width() / 2;
    imu_draw_center_y = tft.height() / 2;
    imu_draw_x0 = imu_draw_center_x - (int16_t)x;
    imu_draw_y0 = imu_draw_center_y - (int16_t)y;
    imu_draw_x1 = imu_draw_center_x + (int16_t)x;
    imu_draw_y1 = imu_draw_center_y + (int16_t)y;

    tft.drawLine(imu_draw_x0, imu_draw_y0, imu_draw_x1, imu_draw_y1, ST7735_WHITE);
    tft.drawCircle(imu_draw_x1, imu_draw_y1, 5, ST7735_WHITE);
}


// 
// Motor menu
// 


void drive_rover_forward(double speed_cps)
{
    update_setpointA(speed_cps);  // cm per s
    update_setpointB(speed_cps);  // cm per s
}

void rotate_rover(double speed_cps)
{
    update_setpointA(speed_cps);  // cm per s
    update_setpointB(-speed_cps);  // cm per s
}

void draw_motors_menu()
{
    int y_offset = TOP_BAR_H + 5;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("A: " + String(encA_pos) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("B: " + String(encB_pos) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("sA: " + String(enc_speedA) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("sB: " + String(enc_speedB) + "   ");  // y_offset += ROW_SIZE;
}

// 
// Safety menu
// 
void draw_safety_menu()
{
    int y_offset = TOP_BAR_H + 5;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("safe: " + String(is_safe_to_move()) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("front: " + String(is_obstacle_in_front()) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("back: " + String(is_obstacle_in_back()) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("tof f: " + String(measure1.RangeMilliMeter) + "   ");  y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("back f: " + String(measure2.RangeMilliMeter) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("fsr f: " + String(fsr_1_val) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("fsr b: " + String(fsr_2_val) + "   "); // y_offset += ROW_SIZE;
}

// 
// Menu events
// 

void down_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: MAIN_MENU_SELECT_INDEX += 1; break;
        case MOTORS_MENU: drive_rover_forward(-900.0); break;
        default: break;
    }
}

void up_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: MAIN_MENU_SELECT_INDEX -= 1; break;
        case MOTORS_MENU: drive_rover_forward(900.0); break;
        default: break;
    }
}

void left_menu_event() {
    switch (DISPLAYED_MENU) {
        case MOTORS_MENU: rotate_rover(-450.0); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}

void right_menu_event() {
    switch (DISPLAYED_MENU) {
        case MOTORS_MENU: rotate_rover(450.0); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}

void enter_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: DISPLAYED_MENU = MAIN_MENU_ENUM_MAPPING[MAIN_MENU_SELECT_INDEX]; break;
        case MOTORS_MENU: drive_rover_forward(0.0); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}
void back_menu_event() {
    if (DISPLAYED_MENU != MAIN_MENU) {
        DISPLAYED_MENU = MAIN_MENU;
    }
}

void screen_change_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: PREV_MAIN_MENU_SELECT_INDEX = -1; break;  // force a redraw of the menu list when switching
        case IMU_MENU: imu_draw_prev_angle += 1; break;  // force compass redraw
        default: break;
        // add new menu entry callbacks (if needed)
    }
}

void draw_menus()
{
    if (PREV_DISPLAYED_MENU != DISPLAYED_MENU) {
        tft.fillScreen(ST7735_BLACK);
        screen_change_event();
    }
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: draw_main_menu(); break;
        case IMU_MENU: draw_imu_menu(); break;
        case MOTORS_MENU: draw_motors_menu(); break;
        case SAFETY_MENU: draw_safety_menu(); break;
        default: break;
        // add new menu entry callbacks
    }
    PREV_DISPLAYED_MENU = DISPLAYED_MENU;

    draw_topbar();
}

#endif  // ROVER6_MENUS