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
        case 0x20df: println_info("IR: SETUP"); break;  // SETUP
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


float setpoint_parse_float = 0.0;
float pid_k_parse_float = 0.0;
int servo_num_parse_int = 0;
int servo_pos_parse_int = 0;
int lox_parse_int = 0;
// method header defined in rover6_serial.h
void process_serial_subfield(uint32_t packet_num, char packet_category, char subfield_type, subfield* field, uint16_t field_index)
{
    switch (packet_category)
    {
        case 1:  // toggle active command
            if (subfield_type != 'c' || field_index != 0) {
                return;
            }

            if (field->c == 1) {
                set_active(true);
            }
            else if (field->c == 2) {
                 set_active(false);
            }
            else if (field->c == 3) {
                soft_restart();  // soft reset the microcontroller
            }
            break;
        case 2:  // get ready message
            if (subfield_type != 's' || field_index != 0) {
                return;
            }
            if (strcmp(field->s, "rover6") == 0) {
                print_data(1, "ls", CURRENT_TIME, "hana");
            }
            break;
        case 3:  // toggle reporting
            if (subfield_type != 'c' || field_index != 0) {
                return;
            }
            if (field->c == 1) {
                rover_state.is_reporting_enabled = true;
            }
            else if (field->c == 2) {
                rover_state.is_reporting_enabled = false;
            }
            else if (field->c == 3) {
                reset();  // reset reporting sensors
            }
            break;
        case 4:  // update time string
            if (subfield_type != 's' || field_index != 0) {
                return;
            }
            
            prev_date_str_update = CURRENT_TIME;
            rpi_date_str = String(field->s);
            break;
        case 5:  // set motor pid setpoints
            if (subfield_type != 'f') {
                return;
            }
            if (field_index == 0) {
                update_setpointA(field->f);
            }
            else if (field_index == 1) {
                update_setpointB(field->f);
            }
        case 6:  //set motor pid constants
            if (subfield_type != 'f') {
                return;
            }
            switch (field_index)
            {
                case 0: set_Kp_A(field->f); break;
                case 1: set_Ki_A(field->f); break;
                case 2: set_Kd_A(field->f); break;
                case 3: set_Kp_B(field->f); break;
                case 4: set_Ki_B(field->f); break;
                case 5: set_Kd_B(field->f); break;
            }
            break;
        case 7:  // set servo position
            if (subfield_type != 'i') {
                return;
            }
            set_servo(field->i >> 8, field->i & 0xff);
            break;
        case 8:  // set servo default position
            if (subfield_type != 'i') {
                return;
            }
            set_servo(field->i & 0xff);
            break;
        case 9:  // set tof safety thresholds
            if (subfield_type != 'i') {
                return;
            }
            switch (field_index)
            {
                case 0: LOX_FRONT_OBSTACLE_UPPER_THRESHOLD_MM = field->i; break;
                case 1: LOX_BACK_OBSTACLE_UPPER_THRESHOLD_MM = field->i; break;
                case 2: LOX_FRONT_OBSTACLE_LOWER_THRESHOLD_MM = field->i; break;
                case 3: LOX_BACK_OBSTACLE_LOWER_THRESHOLD_MM = field->i; break;
            }
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
    initialize_display();  tft.print("Waiting for USB serial...\n");

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

void loop() {
    // current_time = millis();

    read_all_serial();
    report_data();
    update_display();
    update_speed_pid();
    check_motor_timeout();
    // println_info("safety: %d, %d, %d, %d, %d", safety_struct.voltage_ok, safety_struct.are_servos_active, safety_struct.are_motors_active, safety_struct.is_front_tof_ok, safety_struct.is_back_tof_ok);
}
