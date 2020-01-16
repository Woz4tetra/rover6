

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

#include <PID_v1.h>

#define FRONT_TILTER_SERVO_NUM 0
#define BACK_TILTER_SERVO_NUM 1
#define CAMERA_PAN_SERVO_NUM 2
#define CAMERA_TILT_SERVO_NUM 3

int FRONT_TILTER_UP = 70;
int FRONT_TILTER_DOWN = 180;
int BACK_TILTER_UP = 70;
int BACK_TILTER_DOWN = 180;

void set_front_tilter(int angle)
{
    if (angle > FRONT_TILTER_UP) {
        angle = FRONT_TILTER_UP;
    }
    if (angle < FRONT_TILTER_DOWN) {
        angle = FRONT_TILTER_DOWN;
    }
    set_servo(FRONT_TILTER_SERVO_NUM, angle);
}

void set_back_tilter(int angle)
{
    if (angle > BACK_TILTER_UP) {
        angle = BACK_TILTER_UP;
    }
    if (angle < BACK_TILTER_DOWN) {
        angle = BACK_TILTER_DOWN;
    }
    set_servo(BACK_TILTER_SERVO_NUM, angle);
}

#define CAMERA_PAN_UP 0
#define CAMERA_PAN_CENTER 105
#define CAMERA_PAN_DOWN 150
#define CAMERA_TILT_LEFT 90
#define CAMERA_TILT_CENTER 43
#define CAMERA_TILT_RIGHT 0

void center_camera()
{
    set_servo(CAMERA_PAN_SERVO_NUM, CAMERA_PAN_CENTER);
    set_servo(CAMERA_TILT_SERVO_NUM, CAMERA_TILT_CENTER);
}

void set_camera_pan(int angle)
{
    if (angle > CAMERA_PAN_DOWN) {
        angle = CAMERA_PAN_DOWN;
    }
    else if (angle < CAMERA_PAN_UP) {
        angle = CAMERA_PAN_UP;
    }
    set_servo(CAMERA_PAN_SERVO_NUM, angle);
}

void set_camera_tilt(int angle)
{
    if (angle > CAMERA_TILT_LEFT) {
        angle = CAMERA_TILT_LEFT;
    }
    else if (angle < CAMERA_TILT_RIGHT) {
        angle = CAMERA_TILT_RIGHT;
    }
    set_servo(CAMERA_TILT_SERVO_NUM, angle);
}



/*
 * Motor speed controller
 */
//

double speed_setpointA, speed_setpointB = 0.0;  // cm/s
double pid_commandA, pid_commandB = 0.0;  // -255...255
double Kp_A = 200.0, Ki_A = 0.0, Kd_A = 0.0;
double Kp_B = 200.0, Ki_B = 0.0, Kd_B = 0.0;
bool speed_pid_enabled = false;
uint32_t prev_setpointA_time = 0;
uint32_t prev_setpointB_time = 0;
#define SPEED_COMMAND_TIMEOUT_MS 1000

PID motorA_pid(&enc_speedA, &pid_commandA, &speed_setpointA, Kp_A, Ki_A, Kd_A, DIRECT);
PID motorB_pid(&enc_speedB, &pid_commandB, &speed_setpointB, Kp_B, Ki_B, Kd_B, DIRECT);

void set_KAs(double Kp, double Ki, double Kd)
{
    Kp_A = Kp;
    Kp_A = Ki;
    Kp_A = Kd;
}

void set_KBs(double Kp, double Ki, double Kd)
{
    Kp_B = Kp;
    Kp_B = Ki;
    Kp_B = Kd;
}

void setup_pid()
{
    motorA_pid.SetMode(AUTOMATIC);
    motorB_pid.SetMode(AUTOMATIC);
}


void toggle_speed_pid(bool enabled)
{
    if (enabled == speed_pid_enabled) {
        return;
    }
    speed_pid_enabled = enabled;
    if (!speed_pid_enabled) {
        stop_motors();
    }
}

void update_setpointA(double new_setpoint)
{
    speed_setpointA = new_setpoint;
    prev_setpointA_time = CURRENT_TIME;
}

void update_setpointB(double new_setpoint)
{
    speed_setpointB = new_setpoint;
    prev_setpointB_time = CURRENT_TIME;
}

void update_speed_pid()
{
    if (!speed_pid_enabled) {
        return;
    }

    if (CURRENT_TIME - prev_setpointA_time > SPEED_COMMAND_TIMEOUT_MS) {
        update_setpointA(0.0);
    }
    if (CURRENT_TIME - prev_setpointB_time > SPEED_COMMAND_TIMEOUT_MS) {
        update_setpointB(0.0);
    }

    motorA_pid.Compute();
    motorB_pid.Compute();

    // print_info("command:");
    // DATA_SERIAL.print(pid_commandA);
    // DATA_SERIAL.print("\t");
    // DATA_SERIAL.println(pid_commandB);
    set_motorA((int)pid_commandA);
    set_motorB((int)pid_commandB);
}


/*
 * General functions
 */
//

bool is_on_standby = true;

bool read_VL53L0X()
{
    if (is_moving()) {
        lox_samplerate_delay_ms = LOX_SAMPLERATE_FAST_DELAY_MS;
    }
    else {
        lox_samplerate_delay_ms = LOX_SAMPLERATE_SLOW_DELAY_MS;
    }
    if (CURRENT_TIME - lox_report_timer < lox_samplerate_delay_ms) {
        return false;
    }
    lox_report_timer = CURRENT_TIME;
    if (is_moving()) {
        if (is_moving_forward()) {
            // set_front_tilter(FRONT_TILTER_UP);
            // set_back_tilter(BACK_TILTER_DOWN);
            read_front_VL53L0X();
            return true;
        }
        else {
            // set_front_tilter(FRONT_TILTER_DOWN);
            // set_back_tilter(BACK_TILTER_UP);
            read_back_VL53L0X();
            return true;
        }
    }
    else {
        // set_front_tilter(FRONT_TILTER_DOWN);
        // set_back_tilter(BACK_TILTER_DOWN);
        read_front_VL53L0X();
        read_back_VL53L0X();
        return true;
    }
    return false;
}

bool calibrate_safety()
{
    println_info("Calibrating range tilters");
    if (servos_on_standby) {
        println_error("Tilt calibration failed! Servos are on standby");
        is_safe_to_move = false;
        return false;
    }
    set_front_tilter(FRONT_TILTER_DOWN);
    set_back_tilter(BACK_TILTER_DOWN);

    delay(150);

    read_front_VL53L0X();
    read_back_VL53L0X();

    bool success = true;

    if (lox1.Status != VL53L0X_ERROR_NONE) {
        println_error("Tilt calibration failed! lox1 reported error %d", lox1.Status);
        success = false;
    }
    if (lox2.Status != VL53L0X_ERROR_NONE) {
        println_error("Tilt calibration failed! lox2 reported error %d", lox2.Status);
        success = false;
    }

    char* status_string = (char*)"";
    if (measure1.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure1.RangeStatus, status_string);
        println_error("Tilt calibration failed! lox1 measurement reported an error: %s", status_string);
        success = false;
    }
    else {
        if (measure1.RangeMilliMeter > LOX_GROUND_UPPER_THRESHOLD_MM) {
            println_error("Tilt calibration failed! lox1 measurement higher than ground threshold: %d > %d", measure1.RangeMilliMeter, LOX_GROUND_UPPER_THRESHOLD_MM);
            success = false;
        }
        else if (measure1.RangeMilliMeter < LOX_GROUND_LOWER_THRESHOLD_MM) {
            println_error("Tilt calibration failed! lox1 measurement lower than ground threshold: %d < %d", measure1.RangeMilliMeter, LOX_GROUND_LOWER_THRESHOLD_MM);
            success = false;
        }
    }
    if (measure2.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure2.RangeStatus, status_string);
        println_error("Tilt calibration failed! lox2 measurement reported an error: %s", status_string);
        success = false;
    }
    else {
        if (measure2.RangeMilliMeter > LOX_GROUND_UPPER_THRESHOLD_MM) {
            println_error("Tilt calibration failed! lox2 measurement higher than ground threshold: %d > %d", measure2.RangeMilliMeter, LOX_GROUND_UPPER_THRESHOLD_MM);
            success = false;
        }
        else if (measure2.RangeMilliMeter < LOX_GROUND_LOWER_THRESHOLD_MM) {
            println_error("Tilt calibration failed! lox2 measurement lower than ground threshold: %d < %d", measure2.RangeMilliMeter, LOX_GROUND_LOWER_THRESHOLD_MM);
            success = false;
        }
    }

    is_safe_to_move = success;
    if (success) {
        println_info("Range tilter calibration successful!");
    }
    else {
        println_error("Range tilter calibration failed!!");
    }

    return success;
}

void check_for_contact()
{
    if (fsr_1_val < FSR_CONTACT_THRESHOLD && fsr_2_val < FSR_CONTACT_THRESHOLD) {
        return;
    }

    toggle_speed_pid(false);
    if (fsr_1_val >= FSR_CONTACT_THRESHOLD) {
        println_info("Contact on the left bumper");
    }
    if (fsr_2_val >= FSR_CONTACT_THRESHOLD) {
        println_info("Contact on the right bumper");
    }
}

void set_standby(bool standby)
{
    if (is_on_standby == standby) {
        return;
    }
    println_info("Setting standby to: %d", standby);
    is_on_standby = standby;
    set_motor_standby(standby);
    set_servo_standby(standby);
    if (standby) {
        is_safe_to_move = false;
        toggle_speed_pid(false);
    }
    else {
        if (calibrate_safety()) {
            toggle_speed_pid(true);
        }
    }
}

void reset()
{
    set_motorA(0);
    set_motorB(0);
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
            set_standby(!is_on_standby);
            break;  // Play/Pause
        case 0x40bf: println_info("IR: VOL+"); break;  // VOL+
        case 0x20df: println_info("IR: SETUP"); break;  // SETUP
        case 0xa05f:
            println_info("IR: ^");
            drive_forward(300.0);  // cm per s
            break;  // ^
        case 0x609f: println_info("IR: MODE"); break;  // MODE
        case 0x10ef:
            println_info("IR: <");
            rotate(100.0);  // cm per s
            break;  // <
        case 0x906f: println_info("IR: ENTER"); break;  // ENTER
        case 0x50af:
            println_info("IR: >");
            rotate(-100.0);  // cm per s
            break;  // >
        case 0x30cf: println_info("IR: 0 10+"); break;  // 0 10+
        case 0xb04f:
            println_info("IR: v");
            drive_forward(-300.0);  // cm per s
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



void begin()
{
    setup_serial();
    setup_i2c();
    initialize_display();
    setup_servos();
    setup_INA219();
    setup_motors();
    reset_encoders();
    setup_VL53L0X();
    setup_fsrs();
    setup_BNO055();
    setup_IR();
}

void check_serial()
{
    if (DATA_SERIAL.available()) {
        char command = DATA_SERIAL.read();

        if (command == '>') {
            set_standby(false);
        }
        else if (command == '<') {
            set_standby(true);
        }
        else if (command == '?') {
            DATA_SERIAL.print("!\n");
        }
        else if (command == ']') {
            is_reporting_enabled = true;
        }
        else if (command == '[') {
            is_reporting_enabled = false;
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
                    data_buffer = DATA_SERIAL.readStringUntil('\n');
                    DATA_SERIAL.println(data_buffer);
                    switch (data_buffer.charAt(0)) {
                        case 'a': set_motorA(data_buffer.substring(1).toFloat()); break;//update_setpointA(data_buffer.substring(1).toFloat()); break;
                        case 'b': set_motorB(data_buffer.substring(1).toFloat()); break;//update_setpointB(data_buffer.substring(1).toFloat()); break;
                        case 'p': toggle_speed_pid(data_buffer.charAt(1) == '1' ? true : false); break;
                    }
                    break;
                case 'k':
                    data_buffer = DATA_SERIAL.readStringUntil('\n');
                    if (data_buffer.charAt(0) == 'a') {
                        switch (data_buffer.charAt(0)) {
                            case 'p':
                                set_KAs(data_buffer.substring(1).toFloat(), Ki_A, Kd_A);
                                break;
                            case 'i':
                                set_KAs(Kp_A, data_buffer.substring(1).toFloat(), Kd_A);
                                break;
                            case 'd':
                                set_KAs(Kp_A, Ki_A, data_buffer.substring(1).toFloat());
                                break;
                        }
                    }
                    else if (data_buffer.charAt(0) == 'b') {
                        switch (data_buffer.charAt(0)) {
                            case 'p':
                                set_KBs(data_buffer.substring(1).toFloat(), Ki_B, Kd_B);
                                break;
                            case 'i':
                                set_KBs(Kp_B, data_buffer.substring(1).toFloat(), Kd_B);
                                break;
                            case 'd':
                                set_KBs(Kp_B, Ki_B, data_buffer.substring(1).toFloat());
                                break;
                        }
                    }

                case 's':
                    data_buffer = DATA_SERIAL.readStringUntil('\n');
                    if (data_buffer.charAt(0) == 't') {
                        // tell servo positions
                    }
                    else {
                        set_servo(
                            data_buffer.substring(0, 2).toInt(),
                            data_buffer.substring(2).toInt()
                        );
                    }
                    break;

                default: break;
            }
        }
    }
}

void display_data()
{
    if (CURRENT_TIME - tft_display_timer < TFT_UPDATE_DELAY_MS) {
        return;
    }
    tft_display_timer = CURRENT_TIME;

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
    tft.print(motors_on_standby);
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
    tft.print(measure1.RangeStatus);
    tft.print(",");
    tft.print(measure2.RangeStatus);
    tft.println("        ");

    tft.print("fsr: ");
    tft.print(fsr_1_val);
    tft.print(",");
    tft.print(fsr_2_val);
    tft.println("        ");
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
        check_for_contact();
    }
    if (read_IR()) {
        report_IR();
        callback_ir();
    }

    display_data();

    // update_speed_pid();
}


void setup() {
    begin();
}

void loop() {
    // current_time = millis();

    check_serial();
    report_data();
}
