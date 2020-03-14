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
int SCREEN_MID_W = 0;
int SCREEN_MID_H = 0;


void init_menus()
{
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds("A", 0, 0, &x1, &y1, &w, &h);
    ROW_SIZE = h + BORDER_OFFSET_H;
    SCREEN_MID_W = tft.width() / 2;
    SCREEN_MID_H = tft.height() / 2;
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
    tft.getTextBounds(rover_rpi_state.date_str, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(SCREEN_MID_W - w / 2, TOP_BAR_H / 2 - h / 2);
    if (CURRENT_TIME - rover_rpi_state.prev_date_str_update > 1000) {
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        if (CURRENT_TIME - rover_rpi_state.prev_date_str_update > 5000) {
            tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        }
    }

    tft.print(rover_rpi_state.date_str);
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

void draw_prompt(String title, int x0, int y0, int row_size, uint8_t entry_count, ...)
{
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);

    int y_row = y0;
    tft.setCursor(SCREEN_MID_W - w / 2, y_row);
    tft.print(title);
    y_row += row_size * 2;

    va_list args;
    va_start(args, entry_count);
    for (size_t i = 0; i < entry_count; i++) {
        char* entry_text = va_arg(args, char*);
        tft.setCursor(x0, y_row);
        tft.print(entry_text);
        y_row += row_size;
    }
    va_end(args);
}

//
// Main menu
//

enum menu_names {
    MAIN_MENU,
    IMU_MENU,
    MOTORS_MENU,
    SAFETY_MENU,
    WIFI_MENU,
    CAMERA_MENU,
    LIDAR_MENU,
    SHUTDOWN_MENU,
    NONE_MENU,
    NOTIFICATION_MENU
};

const menu_names MAIN_MENU_ENUM_MAPPING[] PROGMEM = {
    SAFETY_MENU,
    IMU_MENU,
    MOTORS_MENU,
    WIFI_MENU,
    CAMERA_MENU,
    LIDAR_MENU,
    SHUTDOWN_MENU
};

const char* const MAIN_MENU_ENTRIES[] PROGMEM = {
    "Safety Systems",
    "IMU",
    "Motors",
    "Wifi Settings",
    "Camera",
    "LIDAR",
    "Shutdown/restart"
};
const int MAIN_MENU_ENTRIES_LEN = 7;

menu_names DISPLAYED_MENU = MAIN_MENU;
menu_names PREV_DISPLAYED_MENU = NONE_MENU;  // for detecting screen change events

int MAIN_MENU_SELECT_INDEX = 0;
int PREV_MAIN_MENU_SELECT_INDEX = -1;
void draw_main_menu()
{
    if (MAIN_MENU_SELECT_INDEX < 0) {
        MAIN_MENU_SELECT_INDEX = MAIN_MENU_ENTRIES_LEN - 1;
    }
    if (MAIN_MENU_SELECT_INDEX >= MAIN_MENU_ENTRIES_LEN) {
        MAIN_MENU_SELECT_INDEX = 0;
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
        ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_WHITE
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

    imu_draw_center_x = SCREEN_MID_W;
    imu_draw_center_y = SCREEN_MID_H;
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


void drive_rover_forward(double speed_tps)
{
    update_setpointA(speed_tps);  // ticks per s
    update_setpointB(speed_tps);  // ticks per s
}

void rotate_rover(double speed_tps)
{
    update_setpointA(speed_tps);  // ticks per s
    update_setpointB(-speed_tps);  // ticks per s
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
struct safety_diagram {
    const int w = 10;
    const int h = 15;
    const int corner_r = 2;
    int origin_x;
    int origin_y;

    int bar_h = 8;
    int error_bar_w = 8;
    uint16_t max_display_val_mm = 500;
    uint16_t front_bar_color;
    uint16_t back_bar_color;
    int bar_max_w;
    double mm_to_pixels;

    int front_bar_w = 0;
    int front_bar_origin_x;
    int front_bar_origin_y;

    int back_bar_w = 0;
    int back_bar_origin_x;
    int back_bar_origin_y;

    int servo_indicator_r = 20;
    int servo_ind_ball_r = 3;
    double front_servo_angle = -1.0;
    int front_servo_origin_x;
    int front_servo_origin_y;
    int front_servo_x;
    int front_servo_y;

    double back_servo_angle = -1.0;
    int back_servo_origin_x;
    int back_servo_origin_y;
    int back_servo_x;
    int back_servo_y;

    int front_lower_threshold_x;
    int back_lower_threshold_x;
    int front_upper_threshold_x;
    int back_upper_threshold_x;
    int threshold_y;
    int threshold_len = 18;

} sd_vals;

int update_front_lower_threshold_x() {
    return SCREEN_MID_W + sd_vals.w / 2 + (int)(sd_vals.mm_to_pixels * LOX_FRONT_OBSTACLE_LOWER_THRESHOLD_MM);
}

int update_back_lower_threshold_x() {
    return SCREEN_MID_W - sd_vals.w / 2 - (int)(sd_vals.mm_to_pixels * LOX_BACK_OBSTACLE_LOWER_THRESHOLD_MM);
}

int update_front_upper_threshold_x() {
    return SCREEN_MID_W + sd_vals.w / 2 + (int)(sd_vals.mm_to_pixels * LOX_FRONT_OBSTACLE_UPPER_THRESHOLD_MM);
}

int update_back_upper_threshold_x() {
    return SCREEN_MID_W - sd_vals.w / 2 - (int)(sd_vals.mm_to_pixels * LOX_BACK_OBSTACLE_UPPER_THRESHOLD_MM);
}


void init_rover_safety_diagram()
{
    int origin_offset_h = 25;
    sd_vals.origin_x = SCREEN_MID_W;
    sd_vals.origin_y = SCREEN_MID_H + origin_offset_h;
    sd_vals.front_bar_origin_x = SCREEN_MID_W + sd_vals.w / 2;
    sd_vals.front_bar_origin_y = SCREEN_MID_H - sd_vals.bar_h / 2 + origin_offset_h;
    sd_vals.back_bar_origin_x = SCREEN_MID_W - sd_vals.w / 2 - 1;
    sd_vals.back_bar_origin_y = sd_vals.front_bar_origin_y;

    sd_vals.front_servo_origin_x = SCREEN_MID_W + 20;
    sd_vals.front_servo_origin_y = SCREEN_MID_H + 40;
    sd_vals.back_servo_origin_x = SCREEN_MID_W - 20;
    sd_vals.back_servo_origin_y = sd_vals.front_servo_origin_y;

    sd_vals.front_servo_angle = -1.0;
    sd_vals.back_servo_angle = -1.0;

    sd_vals.threshold_y = SCREEN_MID_H - sd_vals.threshold_len / 2 + origin_offset_h;
    sd_vals.front_lower_threshold_x = update_front_lower_threshold_x();
    sd_vals.back_lower_threshold_x = update_back_lower_threshold_x();
    sd_vals.front_upper_threshold_x = update_front_upper_threshold_x();
    sd_vals.back_upper_threshold_x = update_back_upper_threshold_x();

    sd_vals.bar_max_w = tft.width() - sd_vals.front_bar_origin_x;
    sd_vals.mm_to_pixels = (double)sd_vals.bar_max_w / sd_vals.max_display_val_mm;

    tft.fillRoundRect(
        sd_vals.origin_x - sd_vals.w / 2, sd_vals.origin_y - sd_vals.h / 2,
        sd_vals.w, sd_vals.h, sd_vals.corner_r, ST7735_WHITE
    );
}

void draw_tof_sensor_bars()
{
    int new_threshold = update_front_lower_threshold_x();
    if (sd_vals.front_lower_threshold_x != new_threshold) {
        tft.drawFastVLine(sd_vals.front_lower_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_BLACK);
        sd_vals.front_lower_threshold_x = new_threshold;
    }
    new_threshold = update_back_lower_threshold_x();
    if (sd_vals.back_lower_threshold_x != new_threshold) {
        tft.drawFastVLine(sd_vals.back_lower_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_BLACK);
        sd_vals.back_lower_threshold_x = new_threshold;
    }

    new_threshold = update_front_upper_threshold_x();
    if (sd_vals.front_upper_threshold_x != new_threshold) {
        tft.drawFastVLine(sd_vals.front_upper_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_BLACK);
        sd_vals.front_upper_threshold_x = new_threshold;
    }
    new_threshold = update_back_upper_threshold_x();
    if (sd_vals.back_upper_threshold_x != new_threshold) {
        tft.drawFastVLine(sd_vals.back_upper_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_BLACK);
        sd_vals.back_upper_threshold_x = new_threshold;
    }

    tft.fillRect(sd_vals.front_bar_origin_x, sd_vals.front_bar_origin_y,
        sd_vals.front_bar_w, sd_vals.bar_h, ST7735_BLACK);
    tft.fillRect(sd_vals.back_bar_origin_x, sd_vals.back_bar_origin_y,
        sd_vals.back_bar_w, sd_vals.bar_h, ST7735_BLACK);

    if (is_front_ok_VL53L0X()) {
        sd_vals.front_bar_w = (int)(sd_vals.mm_to_pixels * measure1.RangeMilliMeter);
        if (is_obstacle_in_front()) {
            sd_vals.front_bar_color = ST7735_YELLOW;
        }
        else {
            if (measure1.RangeMilliMeter < sd_vals.max_display_val_mm) {
                sd_vals.front_bar_color = ST7735_GREEN;
            }
            else {
                sd_vals.front_bar_color = ST7735_BLUE;
            }
        }

    }
    else {
        sd_vals.front_bar_color = ST7735_RED;
        sd_vals.front_bar_w = sd_vals.error_bar_w;
    }

    if (is_back_ok_VL53L0X()) {
        sd_vals.back_bar_w = -(int)(sd_vals.mm_to_pixels * measure2.RangeMilliMeter);
        if (is_obstacle_in_back()) {
            sd_vals.back_bar_color = ST7735_YELLOW;
        }
        else {
            if (measure2.RangeMilliMeter < sd_vals.max_display_val_mm) {
                sd_vals.back_bar_color = ST7735_GREEN;
            }
            else {
                sd_vals.back_bar_color = ST7735_BLUE;
            }
        }
    }
    else {
        sd_vals.back_bar_color = ST7735_RED;
        sd_vals.back_bar_w = -sd_vals.error_bar_w;
    }

    tft.fillRect(sd_vals.front_bar_origin_x, sd_vals.front_bar_origin_y,
        sd_vals.front_bar_w, sd_vals.bar_h, sd_vals.front_bar_color);
    tft.fillRect(sd_vals.back_bar_origin_x, sd_vals.back_bar_origin_y,
        sd_vals.back_bar_w, sd_vals.bar_h, sd_vals.back_bar_color);

    tft.drawFastVLine(sd_vals.front_lower_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_WHITE);
    tft.drawFastVLine(sd_vals.back_lower_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_WHITE);
    tft.drawFastVLine(sd_vals.front_upper_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_WHITE);
    tft.drawFastVLine(sd_vals.back_upper_threshold_x, sd_vals.threshold_y, sd_vals.threshold_len, ST7735_WHITE);
}

void draw_safety_servo_diagrams()
{
    double new_front_servo_angle = tilter_servo_cmd_to_angle(
        servo_positions[FRONT_TILTER_SERVO_NUM]
    ) * PI / 180.0;
    double new_back_servo_angle = tilter_servo_cmd_to_angle(
        servo_positions[BACK_TILTER_SERVO_NUM]
    ) * PI / 180.0;

    // erase previous and redraw lines if the value changed
    if (new_front_servo_angle != sd_vals.front_servo_angle)
    {
        if (sd_vals.front_servo_angle != -1.0)
        {
            tft.drawLine(
                sd_vals.front_servo_origin_x, sd_vals.front_servo_origin_y,
                sd_vals.front_servo_x, sd_vals.front_servo_y, ST7735_BLACK
            );
            tft.drawCircle(
                sd_vals.front_servo_x, sd_vals.front_servo_y,
                sd_vals.servo_ind_ball_r, ST7735_BLACK
            );
        }

        sd_vals.front_servo_angle = new_front_servo_angle;
        sd_vals.front_servo_x = cos(sd_vals.front_servo_angle) * sd_vals.servo_indicator_r + sd_vals.front_servo_origin_x;
        sd_vals.front_servo_y = sin(sd_vals.front_servo_angle) * sd_vals.servo_indicator_r + sd_vals.front_servo_origin_y;

        tft.drawFastHLine(sd_vals.front_servo_origin_x, sd_vals.front_servo_origin_y, sd_vals.servo_indicator_r, ST7735_WHITE);
        tft.drawFastVLine(sd_vals.front_servo_origin_x, sd_vals.front_servo_origin_y, sd_vals.servo_indicator_r, ST7735_WHITE);

        tft.drawLine(
            sd_vals.front_servo_origin_x, sd_vals.front_servo_origin_y,
            sd_vals.front_servo_x, sd_vals.front_servo_y, ST7735_WHITE
        );
        tft.drawCircle(
            sd_vals.front_servo_x, sd_vals.front_servo_y,
            sd_vals.servo_ind_ball_r, ST7735_WHITE
        );
    }
    if (new_back_servo_angle != sd_vals.back_servo_angle)
    {
        if (sd_vals.back_servo_angle != -1.0)
        {
            tft.drawLine(
                sd_vals.back_servo_origin_x, sd_vals.back_servo_origin_y,
                sd_vals.back_servo_x, sd_vals.back_servo_y, ST7735_BLACK
            );
            tft.drawCircle(
                sd_vals.back_servo_x, sd_vals.back_servo_y,
                sd_vals.servo_ind_ball_r, ST7735_BLACK
            );
        }

        sd_vals.back_servo_angle = new_back_servo_angle;
        sd_vals.back_servo_x = -cos(sd_vals.back_servo_angle) * sd_vals.servo_indicator_r + sd_vals.back_servo_origin_x;
        sd_vals.back_servo_y = sin(sd_vals.back_servo_angle) * sd_vals.servo_indicator_r + sd_vals.back_servo_origin_y;

        tft.drawFastHLine(sd_vals.back_servo_origin_x, sd_vals.back_servo_origin_y, -sd_vals.servo_indicator_r, ST7735_WHITE);
        tft.drawFastVLine(sd_vals.back_servo_origin_x, sd_vals.back_servo_origin_y, sd_vals.servo_indicator_r, ST7735_WHITE);

        tft.drawLine(
            sd_vals.back_servo_origin_x, sd_vals.back_servo_origin_y,
            sd_vals.back_servo_x, sd_vals.back_servo_y, ST7735_WHITE
        );
        tft.drawCircle(
            sd_vals.back_servo_x, sd_vals.back_servo_y,
            sd_vals.servo_ind_ball_r, ST7735_WHITE
        );
    }
}

void draw_safety_menu()
{
    int y_offset = TOP_BAR_H + 5;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("tof f: " + String(measure1.RangeMilliMeter) + ", " + String(LOX_FRONT_OBSTACLE_LOWER_THRESHOLD_MM) + ", " + String(LOX_FRONT_OBSTACLE_UPPER_THRESHOLD_MM) + "   ");  y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("tof b: " + String(measure2.RangeMilliMeter) + ", " + String(LOX_BACK_OBSTACLE_LOWER_THRESHOLD_MM) + ", " + String(LOX_BACK_OBSTACLE_UPPER_THRESHOLD_MM) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("fsr l: " + String(fsr_1_val) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("fsr r: " + String(fsr_2_val) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("servo f: " + String(servo_positions[FRONT_TILTER_SERVO_NUM]) + "   "); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("servo b: " + String(servo_positions[BACK_TILTER_SERVO_NUM]) + "   "); // y_offset += ROW_SIZE;

    draw_tof_sensor_bars();
    draw_safety_servo_diagrams();
}

//
// Wifi menu
//
int WIFI_SUBMENU_INDEX = 0;
int PREV_WIFI_SUBMENU_INDEX = 0;
int WIFI_MENU_SELECT_INDEX = 0;
int PREV_WIFI_MENU_SELECT_INDEX = 0;
const int NUM_WIFI_MENU_ENTRIES = 3;

void init_wifi_menu()
{
    tft.fillScreen(ST77XX_BLACK);
    WIFI_SUBMENU_INDEX = 0;
    PREV_WIFI_SUBMENU_INDEX = 0;
    WIFI_MENU_SELECT_INDEX = 0;
    PREV_WIFI_MENU_SELECT_INDEX = -1;
}


void init_hotspot_prompt() {
    tft.fillScreen(ST77XX_BLACK);
}

void draw_hotspot_prompt()
{
    if (WIFI_MENU_SELECT_INDEX < 0) {
        WIFI_MENU_SELECT_INDEX = 0;
    }
    if (WIFI_MENU_SELECT_INDEX >= NUM_WIFI_MENU_ENTRIES) {
        WIFI_MENU_SELECT_INDEX = NUM_WIFI_MENU_ENTRIES - 1;
    }
    if (WIFI_MENU_SELECT_INDEX != PREV_WIFI_MENU_SELECT_INDEX)
    {
        int border_y0 = TOP_BAR_H + 10 + ROW_SIZE * 2 - 1;
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * PREV_WIFI_MENU_SELECT_INDEX + border_y0,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLACK
        );
        draw_prompt("Select hotspot mode", BORDER_OFFSET_W, TOP_BAR_H + 10, ROW_SIZE, NUM_WIFI_MENU_ENTRIES, "Wifi", "Hotspot", "Cancel");
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * WIFI_MENU_SELECT_INDEX + border_y0,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_WHITE
        );
        PREV_WIFI_MENU_SELECT_INDEX = WIFI_MENU_SELECT_INDEX;
    }
}

void draw_main_wifi_menu()
{
    int y_offset = TOP_BAR_H + 5;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("Press enter to set hotspot"); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("IP address: " + rover_rpi_state.ip_address); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset); tft.println("hostname: " + rover_rpi_state.hostname); y_offset += ROW_SIZE;
    tft.setCursor(BORDER_OFFSET_W, y_offset);
    // 0 == unknown, 1 == connected to wifi, 2 == broadcasting hotspot
    switch (rover_rpi_state.broadcasting_hotspot) {
        case 0:  tft.println("No wifi info received!"); break;
        case 1:  tft.println("Connected to wifi     "); break;
        case 2:  tft.println("Broadcasting hotspot  "); break;
        case 3:  tft.println("Disconnected          "); break;
        default:  tft.println("Unknown state: " + String(rover_rpi_state.broadcasting_hotspot)); break;
    }
}


void draw_wifi_menu()
{
    if (WIFI_SUBMENU_INDEX != PREV_WIFI_SUBMENU_INDEX) {
        switch (WIFI_SUBMENU_INDEX) {
            case 0: init_wifi_menu(); break;
            case 1: init_hotspot_prompt(); break;
        }
        PREV_WIFI_SUBMENU_INDEX = WIFI_SUBMENU_INDEX;
    }
    switch (WIFI_SUBMENU_INDEX) {
        case 0: draw_main_wifi_menu(); break;
        case 1: draw_hotspot_prompt(); break;
    }

}

void wifi_menu_enter_event()
{
    if (WIFI_SUBMENU_INDEX == 0) {
        WIFI_SUBMENU_INDEX = 1;
    }
    else if (WIFI_SUBMENU_INDEX == 1) {
        switch (WIFI_MENU_SELECT_INDEX) {
            case 0: print_data("wifi", "d", 1); break;  // "Wifi" selected
            case 1: print_data("wifi", "d", 2); break;  // "Hotspot" selected
            case 2: println_info("Cancel selected "); break;  // "Cancel" selected
            default: break;
        }
        WIFI_SUBMENU_INDEX = 0;
    }
}

bool wifi_menu_back_event()
{
    if (WIFI_SUBMENU_INDEX == 1) {
        WIFI_SUBMENU_INDEX = 0;
        return true;  // displayed menu was a submenu, don't send back to main menu
    }
    return false;
}

//
// Shutdown menu
//

int SHUTDOWN_MENU_SELECT_INDEX = 0;
int PREV_SHUTDOWN_MENU_SELECT_INDEX = -1;
const int NUM_SHUTDOWN_MENU_ENTRIES = 2;
void draw_shutdown_menu()
{
    if (SHUTDOWN_MENU_SELECT_INDEX < 0) {
        SHUTDOWN_MENU_SELECT_INDEX = 0;
    }
    if (SHUTDOWN_MENU_SELECT_INDEX >= NUM_SHUTDOWN_MENU_ENTRIES) {
        SHUTDOWN_MENU_SELECT_INDEX = NUM_SHUTDOWN_MENU_ENTRIES - 1;
    }
    if (SHUTDOWN_MENU_SELECT_INDEX != PREV_SHUTDOWN_MENU_SELECT_INDEX)
    {
        int border_y0 = TOP_BAR_H + 10 + ROW_SIZE * 2 - 1;
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * PREV_SHUTDOWN_MENU_SELECT_INDEX + border_y0,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_BLACK
        );
        draw_prompt("Shutdown?", BORDER_OFFSET_W, TOP_BAR_H + 10, ROW_SIZE, NUM_SHUTDOWN_MENU_ENTRIES, "Yes", "No");
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * SHUTDOWN_MENU_SELECT_INDEX + border_y0,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST7735_WHITE
        );
        PREV_SHUTDOWN_MENU_SELECT_INDEX = SHUTDOWN_MENU_SELECT_INDEX;
    }
}

void shutdown_menu_enter_event()
{
    if (SHUTDOWN_MENU_SELECT_INDEX == 0) {
        print_data("shutdown", "s", "rover6");
    }
    DISPLAYED_MENU = MAIN_MENU;
}

//
// Menu events
//

void down_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: MAIN_MENU_SELECT_INDEX += 1; break;
        case MOTORS_MENU: drive_rover_forward(-170000.0); break;
        case WIFI_MENU: WIFI_MENU_SELECT_INDEX += 1; break;
        case SHUTDOWN_MENU: SHUTDOWN_MENU_SELECT_INDEX += 1; break;
        default: break;
    }
}

void up_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: MAIN_MENU_SELECT_INDEX -= 1; break;
        case MOTORS_MENU: drive_rover_forward(170000.0); break;
        case WIFI_MENU: WIFI_MENU_SELECT_INDEX -= 1; break;
        case SHUTDOWN_MENU: SHUTDOWN_MENU_SELECT_INDEX -= 1; break;
        default: break;
    }
}

void left_menu_event() {
    switch (DISPLAYED_MENU) {
        case MOTORS_MENU: rotate_rover(-85000.0); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}

void right_menu_event() {
    switch (DISPLAYED_MENU) {
        case MOTORS_MENU: rotate_rover(85000.0); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}

void enter_menu_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: DISPLAYED_MENU = MAIN_MENU_ENUM_MAPPING[MAIN_MENU_SELECT_INDEX]; break;
        case MOTORS_MENU: drive_rover_forward(0.0); break;
        case WIFI_MENU: wifi_menu_enter_event(); break;
        case SHUTDOWN_MENU: shutdown_menu_enter_event(); break;
        default: break;
        // add new menu entry callbacks (if needed)
    }
}
void back_menu_event() {
    if (DISPLAYED_MENU == WIFI_MENU && wifi_menu_back_event()) {
        return;
    }
    if (DISPLAYED_MENU != MAIN_MENU) {
        DISPLAYED_MENU = MAIN_MENU;
    }
}

void screen_change_event() {
    switch (DISPLAYED_MENU) {
        case MAIN_MENU: PREV_MAIN_MENU_SELECT_INDEX = -1; break;  // force a redraw of the menu list when switching
        case IMU_MENU: imu_draw_prev_angle += 1; break;  // force compass redraw
        case SAFETY_MENU: init_rover_safety_diagram(); break;  // draw the diagram once
        case WIFI_MENU: init_wifi_menu(); break;
        case SHUTDOWN_MENU: PREV_SHUTDOWN_MENU_SELECT_INDEX = -1;
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
        case WIFI_MENU: draw_wifi_menu(); break;

        case SHUTDOWN_MENU: draw_shutdown_menu(); break;
        default: break;
        // add new menu entry callbacks
    }
    PREV_DISPLAYED_MENU = DISPLAYED_MENU;

    draw_topbar();
}

#endif  // ROVER6_MENUS
