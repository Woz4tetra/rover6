#include <rover6_bno.h>
#include <rover6_encoders.h>
#include <rover6_fsr.h>
#include <rover6_general.h>
#include <rover6_i2c.h>
#include <rover6_ina.h>
#include <rover6_ir_remote.h>
#include <rover6_motors.h>
#include <rover6_serial.h>
#include <rover6_servos.h>
#include <rover6_tft.h>
#include <rover6_tof.h>
#include <rover6_menus.h>
#include <rover6_pid.h>



void set_active(bool active)
{
    if (rover6::rover_state.is_active == active) {
        return;
    }
    rover6_serial::println_info("Setting active to: %d", active);

    rover6::rover_state.is_active = active;
    rover6_motors::set_motors_active(active);
    rover6_servos::set_servos_active(active);
    rover6_pid::set_speed_pid(active);
    rover6_tof::set_lox_active(active);
    rover6_bno::set_bno_active(active);
    if (!active) {
        rover6_motors::stop_motors();
    }
}

void reset()
{
    if (rover6_motors::is_moving()) {
        rover6_motors::stop_motors();
        delay(200);
    }
    rover6_encoders::reset_encoders();
}


void rover6_ir_remote::callback_ir()
{
    if (!ir_result_available) {
        return;
    }

    if (ir_value == 0xffff) {  // 0xffff means repeat last command
        ir_value = prev_ir_value;
    }

    switch (ir_value) {
        case 0x00ff: rover6_serial::println_info("IR: VOL-"); break;  // VOL-
        case 0x807f:
            rover6_serial::println_info("IR: Play/Pause");
            set_active(!rover6::rover_state.is_active);
            break;  // Play/Pause
        case 0x40bf: rover6_serial::println_info("IR: VOL+"); break;  // VOL+
        case 0x20df:
            rover6_serial::println_info("IR: SETUP");
            rover6::soft_restart();
            break;  // SETUP
        case 0xa05f:
            rover6_serial::println_info("IR: ^");
            rover6_menus::up_menu_event();
            break;  // ^
        case 0x609f: rover6_serial::println_info("IR: MODE"); break;  // MODE
        case 0x10ef:
            rover6_serial::println_info("IR: <");
            rover6_menus::left_menu_event();
            break;  // <
        case 0x906f:
            rover6_serial::println_info("IR: ENTER");
            rover6_menus::enter_menu_event();
            break;  // ENTER
        case 0x50af:
            rover6_serial::println_info("IR: >");
            rover6_menus::right_menu_event();
            break;  // >
        case 0x30cf:
            rover6_serial::println_info("IR: 0 10+");
            rover6::rover_state.is_reporting_enabled = !rover6::rover_state.is_reporting_enabled;
            break;  // 0 10+
        case 0xb04f:
            rover6_serial::println_info("IR: v");
            rover6_menus::down_menu_event();
            break;  // v
        case 0x708f:
            rover6_serial::println_info("IR: Del");
            rover6_menus::back_menu_event();
            break;  // Del
        case 0x08f7: rover6_serial::println_info("IR: 1"); break;  // 1
        case 0x8877: rover6_serial::println_info("IR: 2"); break;  // 2
        case 0x48B7: rover6_serial::println_info("IR: 3"); break;  // 3
        case 0x28D7: rover6_serial::println_info("IR: 4"); break;  // 4
        case 0xA857: rover6_serial::println_info("IR: 5"); break;  // 5
        case 0x6897: rover6_serial::println_info("IR: 6"); break;  // 6
        case 0x18E7: rover6_serial::println_info("IR: 7"); break;  // 7
        case 0x9867: rover6_serial::println_info("IR: 8"); break;  // 8
        case 0x58A7: rover6_serial::println_info("IR: 9"); break;  // 9

    }
    // String decode_type;
    // if (irresults->decode_type == NEC) {
    //     decode_type = "NEC";
    // } else if (irresults->decode_type == SONY) {
    //     decode_type = "SONY";
    // } else if (irresults->decode_type == RC5) {
    //     decode_type = "RC5";
    // } else if (irresults->decode_type == RC6) {
    //     decode_type = "RC6";
    // } else if (irresults->decode_type == UNKNOWN) {
    //     decode_type = "???";
    // }

    rover6_ir_remote::ir_result_available = false;
    rover6_ir_remote::ir_type = 0;
    rover6_ir_remote::ir_value = 0;
}

namespace rover6_serial {
    void packet_callback(Rover6Serial* serial_obj, String category, String packet);

    // method header defined in rover6_serial.h
    void info_packet_callback(String category, String packet) {
        packet_callback(info, category, packet);
    }

    void data_packet_callback(String category, String packet) {
        packet_callback(data, category, packet);
    }
}

void rover6_serial::packet_callback(Rover6Serial* serial_obj, String category, String packet)
{
    // rover6_serial::println_info("category: %s, packet: '%s'", category.c_str(), packet.c_str());
    // toggle_active
    if (category.equals("<>")) {
        CHECK_SEGMENT(serial_obj);
        int active_state = serial_obj->get_segment().toInt();
        rover6_serial::println_info("toggle_active %d", active_state);
        switch (active_state)
        {
            case 0: set_active(false); break;
            case 1: set_active(true); break;
            case 2: rover6::soft_restart(); break;
            default:
                break;
        }
    }

    // get_ready
    else if (category.equals("?")) {
        CHECK_SEGMENT(serial_obj);
        if (serial_obj->get_segment().equals("rover6")) {
            rover6_serial::println_info("Received ready signal!");
            ROVER6_SERIAL_WRITE_BOTH("ready", "us", CURRENT_TIME, "dul");
        }
        else {
            rover6_serial::println_error("Invalid ready segment supplied: %s", serial_obj->get_segment().c_str());
        }
    }

    // toggle_reporting
    else if (category.equals("[]")) {
        CHECK_SEGMENT(serial_obj);
        int reporting_state = serial_obj->get_segment().toInt();
        rover6_serial::println_info("toggle_reporting %d", reporting_state);
        switch (reporting_state)
        {
            case 0: rover6::rover_state.is_reporting_enabled = false; break;
            case 1: rover6::rover_state.is_reporting_enabled = true; break;
            case 2: reset(); break;
            default:
                rover6_serial::println_error("Invalid reporting flag received: %d", reporting_state);
                break;
        }
    }

    // rpi_state
    else if (category.equals("rpi")) {
        CHECK_SEGMENT(serial_obj); rover6::rover_rpi_state.ip_address = serial_obj->get_segment();
        CHECK_SEGMENT(serial_obj); rover6::rover_rpi_state.hostname = serial_obj->get_segment();
        CHECK_SEGMENT(serial_obj); rover6::rover_rpi_state.date_str = serial_obj->get_segment();
        rover6::rover_rpi_state.prev_date_str_update = CURRENT_TIME;
        CHECK_SEGMENT(serial_obj); rover6::rover_rpi_state.power_button_state = (bool)serial_obj->get_segment().toInt();
        CHECK_SEGMENT(serial_obj); rover6::rover_rpi_state.broadcasting_hotspot = serial_obj->get_segment().toInt();
    }

    // set_motors
    else if (category.equals("m")) {
        CHECK_SEGMENT(serial_obj); float setpointA = serial_obj->get_segment().toFloat();
        CHECK_SEGMENT(serial_obj); float setpointB = serial_obj->get_segment().toFloat();
        rover6_pid::update_setpointA(setpointA);
        rover6_pid::update_setpointB(setpointB);
    }

    // set_pid_ks
    else if (category.equals("ks")) {
        for (size_t index = 0; index < NUM_PID_KS; index++) {
            CHECK_SEGMENT(serial_obj); rover6_pid::pid_Ks[index] = serial_obj->get_segment().toFloat();
        }
        rover6_pid::set_Ks();  // sets pid constants based on pid_Ks array
    }

    // set_servo
    else if (category.equals("s")) {
        CHECK_SEGMENT(serial_obj); int n = serial_obj->get_segment().toInt();
        CHECK_SEGMENT(serial_obj); int command = serial_obj->get_segment().toInt();
        rover6_servos::set_servo(n, command);
        rover6_servos::report_servo_pos();
    }

    // set_servo_default
    else if (category.equals("sd")) {
        CHECK_SEGMENT(serial_obj); int n = serial_obj->get_segment().toInt();
        rover6_servos::set_servo(n);
        rover6_servos::report_servo_pos();
    }

    // set_safety_thresholds
    else if (category.equals("safe")) {
        for (size_t index = 0; index < 4; index++) {
            CHECK_SEGMENT(serial_obj); rover6_tof::LOX_THRESHOLDS[index] = serial_obj->get_segment().toInt();
        }
        rover6_tof::set_lox_thresholds();  // sets thresholds based on LOX_THRESHOLDS array
    }
}

void report_data()
{
    if (rover6_bno::read_BNO055()) {
        rover6_bno::report_BNO055();
    }
    if (rover6_tof::read_VL53L0X()) {
        // rover6_tof::report_VL53L0X();
    }
    if (rover6_ina::read_INA219()) {
        rover6_ina::report_INA219();
    }
    if (rover6_encoders::read_encoders()) {
        rover6_encoders::report_encoders();
    }
    if (rover6_fsr::read_fsrs()) {
        // rover6_fsr::report_fsrs();
    }
    if (rover6_ir_remote::read_IR()) {
        rover6_ir_remote::report_IR();
        rover6_ir_remote::callback_ir();
    }

}


void setup()
{
    rover6::init_structs();

    rover6_tft::initialize_display();
    rover6_serial::setup_serial();  tft.print("Serial ready!\n");
    rover6_i2c::setup_i2c();  tft.print("I2C ready!\n");
    rover6_encoders::reset_encoders();  tft.print("Encoders ready!\n");
    rover6_fsr::setup_fsrs();  tft.print("FSRs ready!\n");
    rover6_ir_remote::setup_IR();   tft.print("IR ready!\n");
    rover6_motors::setup_motors();   tft.print("Motors ready!\n");
    rover6_bno::setup_BNO055();   tft.print("BNO055 ready!\n");
    rover6_ina::setup_INA219();   tft.print("INA219 ready!\n");
    rover6_servos::setup_servos();   tft.print("Servos ready!\n");
    rover6_tof::setup_VL53L0X();   tft.print("VL53L0Xs ready!\n");
    rover6_pid::setup_pid();

    set_active(true);
    // set_servos_default();

    rover6_servos::set_front_tilter(FRONT_TILTER_UP);
    rover6_servos::set_back_tilter(BACK_TILTER_UP);
    rover6_servos::center_camera();
    rover6_tft::black_display();
    rover6_menus::init_menus();
}

void loop()
{
    rover6_serial::data->read();
    rover6_serial::info->read();
    report_data();

    rover6_menus::draw_menus();
    rover6_pid::update_speed_pid();
    rover6_motors::check_motor_timeout();
}
