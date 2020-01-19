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

#include <PID_v1.h>

/*
 * Motor speed controller
 */
//

double speed_setpointA, speed_setpointB = 0.0;  // cm/s
double pid_commandA, pid_commandB = 0.0;  // -255...255
double Kp_A = 0.1, Ki_A = 0.0, Kd_A = 0.0;
double Kp_B = 0.1, Ki_B = 0.0, Kd_B = 0.0;
uint32_t prev_setpointA_time = 0;
uint32_t prev_setpointB_time = 0;
#define PID_COMMAND_TIMEOUT_MS 1000
#define PID_UPDATE_DELAY_MS 33  // ~30 Hz
uint32_t prev_pid_time = 0;
double ff_command_A, ff_command_B = 0;  // feed forward commands
double ff_speed_A, ff_speed_B = 0;  // feed forward speed. enc_speed - setpoint
double ff_setpoint_A, ff_setpoint_B = 0.0;  // always zero

PID motorA_pid(&ff_speed_A, &pid_commandA, &ff_setpoint_A, Kp_A, Ki_A, Kd_A, DIRECT);
PID motorB_pid(&ff_speed_B, &pid_commandB, &ff_setpoint_B, Kp_B, Ki_B, Kd_B, DIRECT);

void set_KAs(double Kp, double Ki, double Kd)
{
    Kp_A = Kp;
    Ki_A = Ki;
    Kd_A = Kd;
    motorA_pid.SetTunings(Kp_A, Ki_A, Kd_A);
}

void set_KBs(double Kp, double Ki, double Kd)
{
    Kp_B = Kp;
    Ki_B = Ki;
    Kd_B = Kd;
    motorB_pid.SetTunings(Kp_B, Ki_B, Kd_B);
}

void setup_pid()
{
    motorA_pid.SetMode(AUTOMATIC);
    motorB_pid.SetMode(AUTOMATIC);
}

void update_setpointA(double new_setpoint)
{
    speed_setpointA = new_setpoint;
    ff_command_A = speed_setpointA * cps_to_cmd;
    prev_setpointA_time = CURRENT_TIME;
    // print_info("speed_setpointA: ");
    // DATA_SERIAL.println(speed_setpointA);
}

void update_setpointB(double new_setpoint)
{
    speed_setpointB = new_setpoint;
    ff_command_B = speed_setpointB * cps_to_cmd;
    prev_setpointB_time = CURRENT_TIME;
    // print_info("speed_setpointB: ");
    // DATA_SERIAL.println(speed_setpointB);
}

void update_speed_pid()
{
    if (!rover_state.is_speed_pid_enabled) {
        return;
    }

    if (CURRENT_TIME - prev_pid_time < PID_UPDATE_DELAY_MS) {
        return;
    }
    prev_pid_time = CURRENT_TIME;
    ff_speed_A = enc_speedA - speed_setpointA;
    ff_speed_B = enc_speedB - speed_setpointB;

    // print_info("ff_speed_A: ");
    // DATA_SERIAL.println(ff_speed_A);
    // print_info("ff_speed_B: ");
    // DATA_SERIAL.println(ff_speed_B);

    if (speed_setpointA != 0.0) {
        if (CURRENT_TIME - prev_setpointA_time > PID_COMMAND_TIMEOUT_MS) {
            println_info("PID A setpoint timed out");
            update_setpointA(0.0);
        }
    }
    
    if (speed_setpointB != 0.0) {
        if (CURRENT_TIME - prev_setpointB_time > PID_COMMAND_TIMEOUT_MS) {
            println_info("PID B setpoint timed out");
            update_setpointB(0.0);
        }
    }

    motorA_pid.Compute();
    motorB_pid.Compute();

    // apply feed forward commands
    int sum_commandA = pid_commandA + ff_command_A;
    int sum_commandB = pid_commandB + ff_command_B;

    set_motors(sum_commandA, sum_commandB);
}

void set_speed_pid(bool enabled) {
    rover_state.is_speed_pid_enabled = enabled;
}

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

void drive_forward(double speed_cps)  // speed cm per s
{
    update_setpointA(speed_cps);  // cm per s
    update_setpointB(speed_cps);  // cm per s
}

void rotate(double speed_cps)  // speed cm per s
{
    update_setpointA(-speed_cps);  // cm per s
    update_setpointB(speed_cps);  // cm per s
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
            // set_motors(255, 255);
            // drive_forward(300.0);  // cm per s
            MAIN_MENU_SELECT_INDEX -= 1;
            draw_main_menu();
            break;  // ^
        case 0x609f: println_info("IR: MODE"); break;  // MODE
        case 0x10ef:
            println_info("IR: <");
            // set_motors(-100, 100);
            // rotate(100.0);  // cm per s
            break;  // <
        case 0x906f:
            println_info("IR: ENTER");
            // stop_motors();
            break;  // ENTER
        case 0x50af:
            println_info("IR: >");
            // set_motors(100, -100);
            // rotate(-100.0);  // cm per s
            break;  // >
        case 0x30cf: println_info("IR: 0 10+"); break;  // 0 10+
        case 0xb04f:
            println_info("IR: v");
            // set_motors(-255, -255);
            // drive_forward(-300.0);  // cm per s
            MAIN_MENU_SELECT_INDEX += 1;
            draw_main_menu();
            break;  // v
        case 0x708f: println_info("IR: Del"); break;  // Del
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
void check_serial()
{
    if (!DATA_SERIAL.available()) {
        return;
    }
    data_buffer = DATA_SERIAL.readStringUntil('\n');
    print_info("data_buffer: ");
    DATA_SERIAL.println(data_buffer);
    // char command = DATA_SERIAL.read();
    char command = data_buffer.charAt(0);

    if (command == '>') {
        set_active(true);
    }
    else if (command == '<') {
        set_active(false);
    }
    else if (command == '?') {
        DATA_SERIAL.print("!\n");
    }
    else if (command == ']') {
        rover_state.is_reporting_enabled = true;
    }
    else if (command == '[') {
        rover_state.is_reporting_enabled = false;
    }
    else if (command == 'r') {
        reset();
    }
    else if (command == '+') {
        if (DATA_SERIAL.read() == '!') {  // you sure you want to reset?
            soft_restart();
        }
    }
    else {
        switch (command) {
            case 'm':
                if (data_buffer.length() < 3) {
                    println_error("Serial input less than required length");
                    return;
                }
                
                // segment_int = data_buffer.substring(2).toInt();
                setpoint_parse_float = data_buffer.substring(2).toFloat();
                switch (data_buffer.charAt(1)) {
                    case 'a': update_setpointA(setpoint_parse_float); break;  // set_motors(segment_int, motorA.getSpeed()); break;
                    case 'b': update_setpointB(setpoint_parse_float); break;  // set_motors(motorB.getSpeed(), segment_int); break;
                    // case 'p': set_speed_pid(data_buffer.charAt(2) == '1' ? true : false); break;
                }
                break;
            case 'k':
                if (data_buffer.length() < 4) {
                    println_error("Serial input less than required length");
                    return;
                }
                pid_k_parse_float = data_buffer.substring(3).toFloat();
                if (data_buffer.charAt(1) == 'a') {
                    switch (data_buffer.charAt(2)) {
                        case 'p':
                            set_KAs(pid_k_parse_float, Ki_A, Kd_A);
                            break;
                        case 'i':
                            set_KAs(Kp_A, pid_k_parse_float, Kd_A);
                            break;
                        case 'd':
                            set_KAs(Kp_A, Ki_A, pid_k_parse_float);
                            break;
                    }
                }
                else if (data_buffer.charAt(1) == 'b') {
                    switch (data_buffer.charAt(2)) {
                        case 'p':
                            set_KBs(pid_k_parse_float, Ki_B, Kd_B);
                            break;
                        case 'i':
                            set_KBs(Kp_B, pid_k_parse_float, Kd_B);
                            break;
                        case 'd':
                            set_KBs(Kp_B, Ki_B, pid_k_parse_float);
                            break;
                    }
                }
                break;

            case 's':
                if (data_buffer.length() < 5) {
                    println_error("Serial input less than required length");
                    return;
                }
                switch (data_buffer.charAt(1))
                {
                    case 't': report_servo_pos(); break;
                    case 'p': 
                        servo_num_parse_int = data_buffer.substring(2, 4).toInt();
                        servo_pos_parse_int = data_buffer.substring(4).toInt();
                        set_servo(servo_num_parse_int, servo_pos_parse_int);
                        break;
                    case 'd':
                        servo_num_parse_int = data_buffer.substring(0).toInt();
                        set_servo(servo_num_parse_int);
                        break;
                }
                break;

            case 'l':
                if (data_buffer.length() < 3) {
                    println_error("Serial input less than required length");
                    return;
                }
                lox_parse_int = data_buffer.substring(2).toInt();
                switch (data_buffer.charAt(1))
                {
                    case 'u': LOX_OBSTACLE_UPPER_THRESHOLD_MM = lox_parse_int; break;
                    case 'l': LOX_OBSTACLE_LOWER_THRESHOLD_MM = lox_parse_int; break;
                    default: break;
                }
            default: break;
        }
    }
}

void update_display()
{
    if (CURRENT_TIME - tft_display_timer < TFT_UPDATE_DELAY_MS) {
        return;
    }
    tft_display_timer = CURRENT_TIME;


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

    setup_serial();
    setup_i2c();
    initialize_display();
    reset_encoders();  tft.print("Encoders ready!\n");
    setup_fsrs();  tft.print("FSRs ready!\n");
    setup_IR();   tft.print("IR ready!\n");
    setup_motors();   tft.print("Motors ready!\n");
    setup_BNO055();   tft.print("BNO055 ready!\n");
    setup_INA219();   tft.print("INA219 ready!\n");
    setup_servos();   tft.print("Servos ready!\n");
    setup_VL53L0X();   tft.print("VL53L0Xs ready!\n");
    setup_pid();

    set_active(true);
    // set_servos_default();

    set_front_tilter(FRONT_TILTER_UP);
    set_back_tilter(BACK_TILTER_UP);
    center_camera();
    tft.fillScreen(ST77XX_BLACK);
    init_menus();
    draw_main_menu();
}

void loop() {
    // current_time = millis();

    check_serial();
    report_data();
    update_display();
    update_speed_pid();
    check_motor_timeout();
    // println_info("safety: %d, %d, %d, %d, %d", safety_struct.voltage_ok, safety_struct.are_servos_active, safety_struct.are_motors_active, safety_struct.is_front_tof_ok, safety_struct.is_back_tof_ok);
}
