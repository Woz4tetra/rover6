#include <Arduino.h>
#include <i2c_t3.h>

#include <Adafruit_INA219_Teensy.h>

#include <Adafruit_PWMServoDriverTeensy.h>

#include <Adafruit_VL53L0X_Teensy.h>

#include <TB6612.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include <Encoder.h>

#include <Adafruit_BNO055_Teensy.h>

#include <IRremote.h>


/*
 * Serial devices
 */

#define DATA_SERIAL  Serial5
#define SERIAL_MSG_BUFFER_SIZE 0xff
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
const uint16_t FAST_SAMPLERATE_DELAY_MS = 10;
#define PACKET_END '\n'
bool is_reporting_enabled = false;
String data_buffer;
uint32_t current_time = 0;

void print_data(String name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    String data = String(formats) + "\t";
    while (*formats != '\0') {
        if (*formats == 'd') {
            int i = va_arg(args, int);
            data += String(i);
        }
        else if (*formats == 'l') {
            int32_t s = va_arg(args, int32_t);
            data += String(s);
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            data += s;
        }
        else if (*formats == 'f') {
            double f = va_arg(args, double);
            data += String(f);
        }
        data += "\t";
        ++formats;
    }
    va_end(args);
    data += PACKET_END;
    DATA_SERIAL.print(name + "\t" + data);
}


void print_info(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tINFO\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void print_error(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
    va_end(args);

    DATA_SERIAL.print("msg\tERROR\t");
    DATA_SERIAL.print(SERIAL_MSG_BUFFER);
    DATA_SERIAL.print('\n');
}

void setup_serial()
{
    // DATA_SERIAL.begin(115200);
    DATA_SERIAL.begin(500000);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
    print_info("Rover #6");
    print_info("Serial buses initialized.");
}

/*
 * Soft restart
 */

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

void soft_restart()
{
    DATA_SERIAL.end();  // clears the serial monitor  if used
    SCB_AIRCR = 0x05FA0004;  // write value for restart
}

/*
 * I2C
 */

#define I2C_BUS_1 Wire
#define I2C_BUS_2 Wire1

void setup_i2c()
{
    I2C_BUS_1.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    I2C_BUS_1.setDefaultTimeout(200000); // 200ms
    I2C_BUS_2.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
    I2C_BUS_2.setDefaultTimeout(200000); // 200ms
    print_info("I2C initialized.");
}

/*
 * Adafruit PWM servo driver
 * PCA9685
 */

Adafruit_PWMServoDriver servos(0x40, &I2C_BUS_2);


#define NUM_SERVOS 16
#define SERVO_STBY 24

int servo_pulse_mins[NUM_SERVOS];
int servo_pulse_maxs[NUM_SERVOS];
int servo_positions[NUM_SERVOS];

bool servos_on_standby = false;

void set_servo_standby(bool standby)
{
    servos_on_standby = standby;
    if (standby) {  // set servos to low power
        servos.sleep();
    }
    else {  // bring servos out of standby mode
        servos.wakeup();
    }
}

void setup_servos()
{
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_mins[i] = 150;
    }
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_maxs[i] = 600;
    }

    servos.begin();
    servos.setPWMFreq(60);
    delay(10);
    print_info("PCA9685 Servos initialized.");
    pinMode(SERVO_STBY, OUTPUT);
    digitalWrite(SERVO_STBY, LOW);

    set_servo_standby(true);
}

void set_servo(uint8_t n, int angle)
{
    if (servo_positions[n] != angle) {
        servo_positions[n] = angle;
        uint16_t pulse = (uint16_t)map(angle, 0, 180, servo_pulse_mins[n], servo_pulse_maxs[n]);
        servos.setPWM(n, 0, pulse);
    }
}

#define FRONT_TILTER_UP 75
#define FRONT_TILTER_DOWN 160
#define BACK_TILTER_UP 75
#define BACK_TILTER_DOWN 160

void set_front_tilter(int angle)
{
    if (angle > FRONT_TILTER_UP) {
        angle = FRONT_TILTER_UP;
    }
    if (angle < FRONT_TILTER_DOWN) {
        angle = FRONT_TILTER_DOWN;
    }
    set_servo(0, angle);
}

void set_back_tilter(int angle)
{
    if (angle > BACK_TILTER_UP) {
        angle = BACK_TILTER_UP;
    }
    if (angle < BACK_TILTER_DOWN) {
        angle = BACK_TILTER_DOWN;
    }
    set_servo(1, angle);
}

/*
 * Adafruit High-side current and voltage meter
 * INA219
 */

Adafruit_INA219 ina219;
float ina219_shuntvoltage = 0.0;
float ina219_busvoltage = 0.0;
float ina219_current_mA = 0.0;
float ina219_loadvoltage = 0.0;
float ina219_power_mW = 0.0;
uint32_t ina_report_timer = 0;

#define INA_SAMPLERATE_DELAY_MS 1000

void setup_INA219()
{
    ina219.begin(&I2C_BUS_1);
    print_info("INA219 initialized.");
}

bool read_INA219()
{
    if (current_time - ina_report_timer < INA_SAMPLERATE_DELAY_MS) {
        return false;
    }
    ina219_shuntvoltage = ina219.getShuntVoltage_mV();
    ina219_busvoltage = ina219.getBusVoltage_V();
    ina219_current_mA = ina219.getCurrent_mA();
    ina219_power_mW = ina219.getPower_mW();
    ina219_loadvoltage = ina219_busvoltage + (ina219_shuntvoltage / 1000);
    return true;
}
void report_INA219()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("ina", "lfff", millis(), ina219_current_mA, ina219_power_mW, ina219_loadvoltage);
}


/*
 * Adafruit dual motor driver breakout
 * TB6612
 */

#define MOTOR_STBY 26
#define MOTORA_DR1 27
#define MOTORA_DR2 28
#define MOTORB_DR1 31
#define MOTORB_DR2 32
#define MOTORA_PWM 29
#define MOTORB_PWM 30

#define MOTORA_ENCA 23
#define MOTORA_ENCB 22
#define MOTORB_ENCA 21
#define MOTORB_ENCB 20

bool motors_on_standby = false;
bool is_safe_to_move = false;

TB6612 motorA(MOTORA_PWM, MOTORA_DR2, MOTORA_DR1);
TB6612 motorB(MOTORB_PWM, MOTORB_DR1, MOTORB_DR2);

void set_motor_standby(bool standby)
{
    motors_on_standby = standby;
    if (standby) {  // set motors to low power
        digitalWrite(MOTOR_STBY, LOW);
    }
    else {  // bring motors out of standby mode
        digitalWrite(MOTOR_STBY, HIGH);
    }
}

void setup_motors()
{
    pinMode(MOTOR_STBY, OUTPUT);
    motorA.begin();
    motorB.begin();
    print_info("Motors initialized.");
    set_motor_standby(true);
}

void set_motorA(int speed) {
    if (is_safe_to_move || speed == 0) {
        motorA.setSpeed(speed);
    }
}
void set_motorB(int speed) {
    if (is_safe_to_move || speed == 0) {
        motorB.setSpeed(speed);
    }
}

bool is_moving() {
    return motorA.getSpeed() != 0 || motorB.getSpeed() != 0;
}
bool is_moving_forward() {
    return motorA.getSpeed() + motorB.getSpeed() >= 0;
}

/*
 * Encoders
 */

Encoder motorA_enc(MOTORA_ENCA, MOTORA_ENCB);
Encoder motorB_enc(MOTORB_ENCA, MOTORB_ENCB);

long encA_pos = 0;
long encB_pos = 0;

void reset_encoders()
{
    encA_pos = 0;
    encB_pos = 0;
    motorA_enc.write(0);
    motorB_enc.write(0);
}

bool read_encoders()
{
    long new_encA_pos = motorA_enc.read();
    long new_encB_pos = motorB_enc.read();

    if (encA_pos != new_encA_pos || encB_pos != new_encB_pos) {
        encA_pos = new_encA_pos;
        encB_pos = new_encB_pos;

        return true;
    }
    else {
        return false;
    }
}

void report_encoders()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("enc", "lll", millis(), encA_pos, encB_pos);
}

/*
 * Adafruit TOF distance sensor
 * VL53L0X
 */

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// shutdown pins
#define SHT_LOX1 7
#define SHT_LOX2 5

Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

uint32_t lox_report_timer = 0;
#define LOX_SAMPLERATE_FAST_DELAY_MS 150
#define LOX_SAMPLERATE_SLOW_DELAY_MS 500
unsigned int lox_samplerate_delay_ms = LOX_SAMPLERATE_FAST_DELAY_MS;

#define LOX_GROUND_UPPER_THRESHOLD_MM 80
#define LOX_GROUND_LOWER_THRESHOLD_MM 10


void setup_VL53L0X()
{
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    print_info("Shutdown pins inited...");

    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    print_info("Both in reset mode...(pins are low)");
    delay(10);
    print_info("Starting...");

    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // initing LOX1
    if(!lox1.begin(LOX1_ADDRESS, false, &I2C_BUS_1)) {
        print_error("Failed to boot first VL53L0X");
        while(1);
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if(!lox2.begin(LOX2_ADDRESS, false, &I2C_BUS_1)) {
        print_error("Failed to boot second VL53L0X");
        while(1);
    }
    print_info("VL53L0X's initialized.");
}
void read_front_VL53L0X() {
    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
}

void read_back_VL53L0X() {
    lox2.rangingTest(&measure2, false);
}


void report_VL53L0X()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("lox", "ldddddd", millis(),
        measure1.RangeMilliMeter, measure2.RangeMilliMeter,
        measure1.RangeStatus, measure2.RangeStatus,
        lox1.Status, lox2.Status  // lookup table in vl53l0x_def.h line 133
    );
}

/*
 * Adafruit TFT 1.8" display
 * ST7735
 */

#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_LITE   6
const float TFT_PI = 3.1415926;
#define TFT_BUFFER_SIZE  0xff
char TFT_MSG_BUFFER[TFT_BUFFER_SIZE];


Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
uint8_t tft_brightness;

void set_display_brightness(int brightness)
{
    analogWrite(TFT_LITE, brightness);
    tft_brightness = brightness;
}

void initialize_display()
{
    pinMode(TFT_LITE, OUTPUT);
    tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    delay(10);
    set_display_brightness(255);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    tft.setTextWrap(false);
    tft.setTextSize(1);
    tft.setRotation(1); // horizontal display

    print_info("TFT display initialized.");
    tft.print("Hello!");
}


void print_display(const char* message, ...)
{
    va_list args;
    va_start(args, message);
    vsnprintf(TFT_MSG_BUFFER, TFT_BUFFER_SIZE, message, args);
    va_end(args);

    tft.print(TFT_MSG_BUFFER);
}


/*
 * Adafruit FSR
 * Interlink 402
 */

#define FSR_PIN_1 35
#define FSR_PIN_2 36

uint16_t fsr_1_val;
uint16_t fsr_2_val;
uint32_t fsr_report_timer = 0;
#define FSR_SAMPLERATE_DELAY_MS 250

void setup_fsrs()
{
    pinMode(FSR_PIN_1, INPUT);
    pinMode(FSR_PIN_2, INPUT);
    print_info("FSRs initialized.");
}

bool read_fsrs()
{
    if (current_time - fsr_report_timer < FSR_SAMPLERATE_DELAY_MS) {
        return false;
    }
    fsr_1_val = analogRead(FSR_PIN_1);
    fsr_2_val = analogRead(FSR_PIN_2);
    return true;
}

void report_fsrs()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("fsr", "ldd", millis(), fsr_1_val, fsr_2_val);
}

/*
 * Adafruit 9-DOF Absolute Orientation IMU
 * BNO055
 */

#define BNO055_RST_PIN 25
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
#define BNO055_DATA_BUF_LEN 9

sensors_event_t orientationData;
sensors_event_t angVelocityData;
sensors_event_t linearAccelData;
int8_t bno_board_temp;
Adafruit_BNO055 bno(-1, BNO055_ADDRESS_A, &I2C_BUS_2);
bool is_bno_setup = false;

uint32_t bno_report_timer = 0;
#define BNO_SAMPLERATE_DELAY_MS 100

void setup_BNO055()
{
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        print_error("No BNO055 detected!! Check your wiring or I2C address");
    }
    else {
        delay(500);
        is_bno_setup = true;
        print_info("BNO055 initialized.");
    }
}

bool read_BNO055()
{
    if (!is_bno_setup) {
        return false;
    }

    if (current_time - bno_report_timer < BNO_SAMPLERATE_DELAY_MS) {
        return false;
    }
    bno_report_timer = current_time;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    return true;
}

void report_BNO055()
{
    if (!is_reporting_enabled) {
        return;
    }

    print_data(
        "bno", "lfffffffff",
        millis(),
        orientationData.orientation.x,
        orientationData.orientation.y,
        orientationData.orientation.z,
        angVelocityData.gyro.x,
        angVelocityData.gyro.y,
        angVelocityData.gyro.z,
        linearAccelData.acceleration.x,
        linearAccelData.acceleration.y,
        linearAccelData.acceleration.z
    );
}

/*
 * IR remote receiver
 */

#define IR_RECEIVER_PIN 2

IRrecv irrecv(IR_RECEIVER_PIN);
decode_results irresults;
bool ir_result_available = false;
uint8_t ir_type = 0;
uint16_t ir_value = 0;
uint16_t prev_ir_value = 0;

void setup_IR()
{
    irrecv.enableIRIn();
    irrecv.blink13(false);
}

void callback_ir();

bool read_IR()
{
    if (irrecv.decode(&irresults)) {
        ir_result_available = true;
        ir_type = irresults.decode_type;
        prev_ir_value = ir_value;
        ir_value = irresults.value;
        irrecv.resume(); // Receive the next value
        callback_ir();
        print_info("IR: %d", ir_value);
        return true;
    }
    else {
        return false;
    }
}
void report_IR()
{
    if (!is_reporting_enabled) {
        return;
    }
    print_data("irr", "ldd", millis(), ir_type, ir_value);
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
    if (current_time - lox_report_timer < lox_samplerate_delay_ms) {
        return false;
    }
    lox_report_timer = current_time;
    if (is_moving()) {
        if (is_moving_forward()) {
            set_front_tilter(FRONT_TILTER_UP);
            set_back_tilter(BACK_TILTER_DOWN);
            read_front_VL53L0X();
            return true;
        }
        else {
            set_front_tilter(FRONT_TILTER_DOWN);
            set_back_tilter(BACK_TILTER_UP);
            read_back_VL53L0X();
            return true;
        }
    }
    else {
        set_front_tilter(FRONT_TILTER_DOWN);
        set_back_tilter(BACK_TILTER_DOWN);
        read_front_VL53L0X();
        read_back_VL53L0X();
        return true;
    }
    return false;
}

bool calibrate_tilters()
{
    print_info("Calibrating range tilters");
    if (servos_on_standby) {
        print_error("Tilt calibration failed! Servos are on standby");
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
        print_error("Tilt calibration failed! lox1 reported error %d", lox1.Status);
        success = false;
    }
    if (lox2.Status != VL53L0X_ERROR_NONE) {
        print_error("Tilt calibration failed! lox2 reported error %d", lox2.Status);
        success = false;
    }

    char* status_string = (char*)"";
    if (measure1.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure1.RangeStatus, status_string);
        print_error("Tilt calibration failed! lox1 measurement reported an error: %s", status_string);
        success = false;
    }
    else {
        if (measure1.RangeMilliMeter > LOX_GROUND_UPPER_THRESHOLD_MM) {
            print_error("Tilt calibration failed! lox1 measurement higher than ground threshold: %d > %d", measure1.RangeMilliMeter, LOX_GROUND_UPPER_THRESHOLD_MM);
            success = false;
        }
        else if (measure1.RangeMilliMeter < LOX_GROUND_LOWER_THRESHOLD_MM) {
            print_error("Tilt calibration failed! lox1 measurement lower than ground threshold: %d < %d", measure1.RangeMilliMeter, LOX_GROUND_LOWER_THRESHOLD_MM);
            success = false;
        }
    }
    if (measure2.RangeStatus != 0) {
        VL53L0X_get_range_status_string(measure2.RangeStatus, status_string);
        print_error("Tilt calibration failed! lox2 measurement reported an error: %s", status_string);
        success = false;
    }
    else {
        if (measure2.RangeMilliMeter > LOX_GROUND_UPPER_THRESHOLD_MM) {
            print_error("Tilt calibration failed! lox2 measurement higher than ground threshold: %d > %d", measure2.RangeMilliMeter, LOX_GROUND_UPPER_THRESHOLD_MM);
            success = false;
        }
        else if (measure2.RangeMilliMeter < LOX_GROUND_LOWER_THRESHOLD_MM) {
            print_error("Tilt calibration failed! lox2 measurement lower than ground threshold: %d < %d", measure2.RangeMilliMeter, LOX_GROUND_LOWER_THRESHOLD_MM);
            success = false;
        }
    }

    is_safe_to_move = success;
    if (success) {
        print_info("Range tilter calibration successful!");
    }
    else {
        print_error("Range tilter calibration failed!!");
    }

    return success;
}

void set_standby(bool standby)
{
    if (is_on_standby == standby) {
        return;
    }
    print_info("Setting standby to: %d", standby);
    is_on_standby = standby;
    set_motor_standby(standby);
    set_servo_standby(standby);
    if (standby) {
        is_safe_to_move = false;
    }
    else {
        calibrate_tilters();
    }
}

void reset()
{
    set_motorA(0);
    set_motorB(0);
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
        case 0x00ff: print_info("IR: VOL-"); break;  // VOL-
        case 0x807f:
            print_info("IR: Play/Pause");
            set_standby(!is_on_standby);
            break;  // Play/Pause
        case 0x40bf: print_info("IR: VOL+"); break;  // VOL+
        case 0x20df: print_info("IR: SETUP"); break;  // SETUP
        case 0xa05f: print_info("IR: ^"); break;  // ^
        case 0x609f: print_info("IR: MODE"); break;  // MODE
        case 0x10ef: print_info("IR: <"); break;  // <
        case 0x906f: print_info("IR: ENTER"); break;  // ENTER
        case 0x50af: print_info("IR: >"); break;  // >
        case 0x30cf: print_info("IR: 0 10+"); break;  // 0 10+
        case 0xb04f: print_info("IR: v"); break;  // v
        case 0x708f: print_info("IR: Del"); break;  // Del
        case 0x08f7: print_info("IR: 1"); break;  // 1
        case 0x8877: print_info("IR: 2"); break;  // 2
        case 0x48B7: print_info("IR: 3"); break;  // 3
        case 0x28D7: print_info("IR: 4"); break;  // 4
        case 0xA857: print_info("IR: 5"); break;  // 5
        case 0x6897: print_info("IR: 6"); break;  // 6
        case 0x18E7: print_info("IR: 7"); break;  // 7
        case 0x9867: print_info("IR: 8"); break;  // 8
        case 0x58A7: print_info("IR: 9"); break;  // 9

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
                        case 'a': set_motorA(data_buffer.substring(1).toInt()); break;
                        case 'b': set_motorB(data_buffer.substring(1).toInt()); break;
                        case 's': set_motor_standby((bool)(data_buffer.substring(1).toInt())); break;
                    }
                    break;

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
    tft.setCursor(0, 0);

    tft.print(String(ina219_current_mA));
    tft.print("mA, ");
    tft.print(String(ina219_loadvoltage));
    tft.println("V         ");

    tft.print("B: ");
    tft.print(String(orientationData.orientation.x));
    tft.print(",");
    tft.print(String(orientationData.orientation.y));
    tft.print(",");
    tft.print(String(orientationData.orientation.z));
    tft.println("        ");

    print_display("M: %d, %d, %d         \n\
E: %d, %d         \n\
D: %d, %d, %d, %d         \n\
F: %d, %d         \n",
        motorA.getSpeed(), motorB.getSpeed(), motors_on_standby,
        encA_pos, encB_pos,
        measure1.RangeMilliMeter, measure2.RangeMilliMeter, measure1.RangeStatus, measure2.RangeStatus,
        fsr_1_val, fsr_2_val
    );
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
    }

    display_data();
}


void setup() {
    begin();
}

void loop() {
    current_time = millis();

    check_serial();
    report_data();
}
