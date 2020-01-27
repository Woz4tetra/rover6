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
    if (rover_state.is_active == active) {
        return;
    }
    println_info("Setting active to: %d", active);

    rover_state.is_active = active;
    set_motors_active(active);
    set_servos_active(active);
    set_speed_pid(active);
    set_lox_active(active);
    set_bno_active(active);
    if (!active) {
        stop_motors();
    }   
}

void reset()
{
    stop_motors();
    delay(200);
    reset_encoders();
}


void callback_ir()
{
    if (!ir_result_available) {
        return;
    }

    if (ir_value == 0xffff) {  // 0xffff means repeat last command
        ir_value = prev_ir_value;
    }

    switch (ir_value) {
        case 0x00ff: println_info("IR: VOL-"); break;  // VOL-
        case 0x807f:
            println_info("IR: Play/Pause");
            set_active(!rover_state.is_active);
            break;  // Play/Pause
        case 0x40bf: println_info("IR: VOL+"); break;  // VOL+
        case 0x20df:
            println_info("IR: SETUP");
            soft_restart();
            break;  // SETUP
        case 0xa05f:
            println_info("IR: ^");
            up_menu_event();
            break;  // ^
        case 0x609f: println_info("IR: MODE"); break;  // MODE
        case 0x10ef:
            println_info("IR: <");
            left_menu_event();
            break;  // <
        case 0x906f:
            println_info("IR: ENTER");
            enter_menu_event();
            break;  // ENTER
        case 0x50af:
            println_info("IR: >");
            right_menu_event();
            break;  // >
        case 0x30cf:
            println_info("IR: 0 10+");
            rover_state.is_reporting_enabled = !rover_state.is_reporting_enabled;
            break;  // 0 10+
        case 0xb04f:
            println_info("IR: v");
            down_menu_event();
            break;  // v
        case 0x708f:
            println_info("IR: Del");
            back_menu_event();
            break;  // Del
        case 0x08f7: println_info("IR: 1"); break;  // 1
        case 0x8877: println_info("IR: 2"); break;  // 2
        case 0x48B7: println_info("IR: 3"); break;  // 3
        case 0x28D7: println_info("IR: 4"); break;  // 4
        case 0xA857: println_info("IR: 5"); break;  // 5
        case 0x6897: println_info("IR: 6"); break;  // 6
        case 0x18E7: println_info("IR: 7"); break;  // 7
        case 0x9867: println_info("IR: 8"); break;  // 8
        case 0x58A7: println_info("IR: 9"); break;  // 9

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

    ir_result_available = false;
    ir_type = 0;
    ir_value = 0;
}

// method header defined in rover6_serial.h
void process_serial_packet(String category, String packet)
{
    println_info("category: %s, packet: '%s'", category.c_str(), packet.c_str());
    // toggle_active
    if (category.equals("<>")) {
        if (get_segment(packet, &current_segment, &current_packet_index)) {
            println_info("toggle_active %d", current_segment.toInt());
            switch (current_segment.toInt())
            {
                case 0: set_active(false); break;
                case 1: set_active(true); break;
                case 2: soft_restart(); break;
                default:
                    break;
            }
        }
        else {
            println_error("Not enough segments supplied for toggle_active: %s", packet.c_str());
        }
    }

    // get_ready
    else if (category.equals("?")) {
        if (get_segment(packet, &current_segment, &current_packet_index)) {
            if (current_segment.equals("rover6")) {
                println_info("Received ready signal!");
                print_data("ready", "us", CURRENT_TIME, "hana");
            }
            else {
                println_error("Invalid ready segment supplied: %s", current_segment.c_str());
            }
        }
        else {
            println_error("Not enough segments supplied for get_ready: %s", packet.c_str());
        }
    }

    // toggle_reporting
    else if (category.equals("[]")) {
        if (get_segment(packet, &current_segment, &current_packet_index)) {
            switch (current_segment.toInt())
            {
                case 0: rover_state.is_reporting_enabled = false; break;
                case 1: rover_state.is_reporting_enabled = true; break;
                case 2: reset(); break;
                default:
                    break;
            }
        }
        else {
            println_error("Not enough segments supplied for toggle_reporting: %s", packet.c_str());
        }
    }

    // update_time_str
    else if (category.equals("t")) {
        if (get_segment(packet, &current_segment, &current_packet_index)) {
            prev_date_str_update = CURRENT_TIME;
            rpi_date_str = current_segment;
        }
        else {
            println_error("Not enough segments supplied for update_time_str: %s", packet.c_str());
        }
    }

    // set_motors
    else if (category.equals("m")) {
        if (!get_segment(packet, &current_segment, &current_packet_index)) {
            println_error("Not enough segments supplied for motor command A: %s", packet.c_str());
            return;
        }
        float setpointA = current_segment.toFloat();
        if (!get_segment(packet, &current_segment, &current_packet_index)) {
            println_error("Not enough segments supplied for motor command B: %s", packet.c_str());
            return;
        }
        float setpointB = current_segment.toFloat();
        update_setpointA(setpointA);
        update_setpointB(setpointB);
    }

    // set_pid_ks
    else if (category.equals("ks")) {
        for (size_t index = 0; index < NUM_PID_KS; index++) {
            if (!get_segment(packet, &current_segment, &current_packet_index)) {
                println_error("Not enough segments supplied for set_pid_ks: %s", packet.c_str());
                return;
            }
            pid_Ks[index] = current_segment.toFloat();
        }
        set_Ks();  // sets pid constants based on pid_Ks array
    }

    // set_servo
    else if (category.equals("s")) {
        if (!get_segment(packet, &current_segment, &current_packet_index)) {
            println_error("Not enough segments supplied for set_servo n: %s", packet.c_str());
            return;
        }
        int n = current_segment.toInt();
        if (!get_segment(packet, &current_segment, &current_packet_index)) {
            println_error("Not enough segments supplied for set_servo command: %s", packet.c_str());
            return;
        }
        int command = current_segment.toInt();
        set_servo(n, command);
        report_servo_pos();
    }

    // set_servo_default
    else if (category.equals("sd")) {
        if (!get_segment(packet, &current_segment, &current_packet_index)) {
            println_error("Not enough segments supplied for set_servo_default: %s", packet.c_str());
            return;
        }
        int n = current_segment.toInt();
        set_servo(n);
        report_servo_pos();
    }

    // set_safety_thresholds
    else if (category.equals("safe")) {
        for (size_t index = 0; index < 4; index++) {
            if (!get_segment(packet, &current_segment, &current_packet_index)) {
                println_error("Not enough segments supplied for set_safety_thresholds: %s", packet.c_str());
                return;
            }
            LOX_THRESHOLDS[index] = current_segment.toInt();
        }
        set_lox_thresholds();  // sets thresholds based on LOX_THRESHOLDS array
    }
}

void update_display()
{
    if (CURRENT_TIME - tft_display_timer < TFT_UPDATE_DELAY_MS) {
        return;
    }
    tft_display_timer = CURRENT_TIME;

    draw_menus();
}


void report_data()
{
    if (read_BNO055()) {
        report_BNO055();
    }
    if (read_VL53L0X()) {
        report_VL53L0X();
    }
    if (read_INA219()) {
        report_INA219();
    }
    if (read_encoders()) {
        report_encoders();
    }
    if (read_fsrs()) {
        report_fsrs();
    }
    if (read_IR()) {
        report_IR();
        callback_ir();
    }

}


void setup()
{
    init_structs();

    initialize_display();
    setup_serial();  tft.print("Serial ready!\n");
    setup_i2c();  tft.print("I2C ready!\n");
    reset_encoders();  tft.print("Encoders ready!\n");
    setup_fsrs();  tft.print("FSRs ready!\n");
    setup_IR();   tft.print("IR ready!\n");
    setup_motors();   tft.print("Motors ready!\n");
    setup_BNO055();   tft.print("BNO055 ready!\n");
    setup_INA219();   tft.print("INA219 ready!\n");
    setup_servos();   tft.print("Servos ready!\n");
    setup_VL53L0X();   tft.print("VL53L0Xs ready!\n");
    setup_pid();

    set_active(false);
    // set_servos_default();

    set_front_tilter(FRONT_TILTER_UP);
    set_back_tilter(BACK_TILTER_UP);
    center_camera();
    tft.fillScreen(ST77XX_BLACK);
    init_menus();
}

void loop()
{
    read_all_serial();
    report_data();
    update_display();
    update_speed_pid();
    check_motor_timeout();
}
