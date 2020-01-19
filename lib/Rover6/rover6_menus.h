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

void init_menus()
{
    int16_t  x1, y1;
    uint16_t w, h;
    tft.getTextBounds("A", 0, 0, &x1, &y1, &w, &h);
    ROW_SIZE = h + 3;
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


void draw_main_menu()
{
    tft.drawRect(0, 0, tft.width(), ROW_SIZE, ST7735_BLUE);
    draw_sensor_data();

    // tft.println("Entry 1");
    // tft.println("Entry 2");
    // tft.println("Entry 3");
    // tft.println("Entry 4");
    // tft.println("Entry 5");
    // tft.println("Entry 6");
    // tft.println("Entry 7");
    // tft.println("Entry 8");
    // tft.println("Entry 9");
    // tft.println("Entry 10");
    // tft.println("Entry 11");
    // tft.println("Entry 12");
    // tft.println("Entry 13");
    // tft.println("Entry 14");
    // tft.println("Entry 15");
}