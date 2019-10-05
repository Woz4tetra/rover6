
#include "Rover6.h"

Rover6::Rover6()
{
    servos = new Adafruit_PWMServoDriver(0x40, &Wire1);
    servo_pulse_mins = new int[NUM_SERVOS];
    servo_pulse_maxs = new int[NUM_SERVOS];
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_mins[i] = 150;
    }
    for (size_t i = 0; i < NUM_SERVOS; i++) {
        servo_pulse_maxs[i] = 600;
    }

    ina219 = new Adafruit_INA219();
    ina219_shuntvoltage = 0.0;
    ina219_busvoltage = 0.0;
    ina219_current_mA = 0.0;
    ina219_loadvoltage = 0.0;
    ina219_power_mW = 0.0;

    motorA = new TB6612(MOTORA_PWM, MOTORA_DR1, MOTORA_DR2);
    motorB = new TB6612(MOTORB_PWM, MOTORB_DR1, MOTORB_DR2);
    //encoder objects initialized in setup
    encA_pos = 0;
    encB_pos = 0;

    lox1 = new Adafruit_VL53L0X();
    lox2 = new Adafruit_VL53L0X();
    measure1 = new VL53L0X_RangingMeasurementData_t();
    measure2 = new VL53L0X_RangingMeasurementData_t();

    tft = new Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

    bno = new Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire1);
    orientationData = new sensors_event_t();
    angVelocityData = new sensors_event_t();
    linearAccelData = new sensors_event_t();
    bno_board_temp = 0;

    is_idle = true;

    current_time = 0;
    bno_report_timer = 0;
    fast_sensor_report_timer = 0;

    irrecv = new IRrecv(IR_RECEIVER_PIN);
    irresults = new decode_results();
    ir_result_available = false;
}

void Rover6::begin()
{
    setup_serial();
    setup_i2c();
    setup_servos();
    setup_INA219();
    setup_motors();
    setup_encoders();
    setup_VL53L0X();
    setup_fsrs();
    initialize_display();
    setup_BNO055();
    setup_IR();
    // setup_timers();

    set_idle(true);

    current_time = millis();
    bno_report_timer = millis();
    fast_sensor_report_timer = millis();
}

void Rover6::set_idle(bool state)
{
    if (state == is_idle) {
        return;
    }
    is_idle = state;
    if (is_idle) {
        set_motor_standby(true);
        set_display_brightness(0);
        // setup_timers();
        // interrupts();
    }
    else {
        set_motor_standby(false);
        set_display_brightness(255);
        current_time = millis();
        bno_report_timer = millis();
        fast_sensor_report_timer = millis();
        // noInterrupts();
        // end_timers();
    }
}

void Rover6::write(String name, const char *formats, ...)
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

void Rover6::check_serial()
{
    if (DATA_SERIAL.available()) {
        String command = DATA_SERIAL.readStringUntil('\n');

        char first = command.charAt(0);
        if (first == '>') {
            set_idle(false);
        }
        else if (first == '<') {
            set_idle(true);
        }
        else if (first == '?') {
            DATA_SERIAL.print("!\n");
        }
        else if (first == 'r') {
            DATA_SERIAL.println("Reporting");
            report_status();
        }

        if (is_idle) {
            return;
        }

        switch (first) {
            case 'm':
                switch (command.charAt(1)) {
                    case 'a': set_motorA(command.substring(2).toInt()); break;
                    case 'b': set_motorB(command.substring(2).toInt()); break;
                    case 's': set_motor_standby((bool)(command.substring(2).toInt())); break;
                }
                break;
            case 's':
                set_servo(
                    command.substring(2, 3).toInt(),
                    command.substring(3).toInt()
                );
                break;
            case 'd': display_image(command.substring(1)); break;
        }
    }
}

void Rover6::report_data()
{
    if (is_idle) {
        delay(50);
        return;
    }

    current_time = millis();
    if (current_time - bno_report_timer > BNO055_SAMPLERATE_DELAY_MS)
    {
        bno_report_timer = current_time;
        read_BNO055();
        read_VL53L0X();

        report_BNO055();
        report_VL53L0X();
    }
    if (current_time - fast_sensor_report_timer > FAST_SAMPLERATE_DELAY_MS)
    {
        fast_sensor_report_timer = current_time;
        read_INA219();
        read_encoders();
        read_fsrs();
        read_IR();

        report_INA219();
        report_encoders();
        report_fsrs();
        report_IR();
    }
}

void Rover6::report_status()
{
    print_info("VL53L0X report");

    uint8_t system, gyro, accel, mag = 0;
    bno->getCalibration(&system, &gyro, &accel, &mag);

    print_info("BNO055 report:");
    MSG_SERIAL.print("TEMP: ");
    MSG_SERIAL.print(this->bno->getTemp());

    MSG_SERIAL.print("\tCALIBRATION: Sys=");
    MSG_SERIAL.print(system, DEC);
    MSG_SERIAL.print(" Gyro=");
    MSG_SERIAL.print(gyro, DEC);
    MSG_SERIAL.print(" Accel=");
    MSG_SERIAL.print(accel, DEC);
    MSG_SERIAL.print(" Mag=");
    MSG_SERIAL.println(mag, DEC);

    // print last distance measurement with all fields
    // print bno status
    // print current actuator commands
}


void Rover6::setup_serial()
{
    MSG_SERIAL.begin(115200);
    while (!MSG_SERIAL) {
        delay(1);
    }

    // DATA_SERIAL.begin(115200);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
    DATA_SERIAL.begin(500000);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
    print_info("Rover #6");
    print_info("Serial buses initialized.");
}

void Rover6::setup_i2c()
{
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000); // 200ms
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
    Wire1.setDefaultTimeout(200000); // 200ms
    print_info("I2C initialized.");
}

void Rover6::setup_servos()
{
    servos->begin();
    servos->setPWMFreq(60);
    delay(10);
    print_info("PCA9685 Servos initialized.");
}

void Rover6::set_servo(uint8_t n, double angle) {
    uint16_t pulse = (uint16_t)map(angle, 0, 180, servo_pulse_mins[n], servo_pulse_maxs[n]);
    servos->setPWM(n, 0, pulse);
}

void Rover6::setup_INA219()
{
    ina219->begin(&Wire);
    print_info("INA219 initialized.");
}

void Rover6::read_INA219()
{
    ina219_shuntvoltage = ina219->getShuntVoltage_mV();
    ina219_busvoltage = ina219->getBusVoltage_V();
    ina219_current_mA = ina219->getCurrent_mA();
    ina219_power_mW = ina219->getPower_mW();
    ina219_loadvoltage = ina219_busvoltage + (ina219_shuntvoltage / 1000);
}

void Rover6::report_INA219() {
    write("ina", "lfff", millis(), ina219_current_mA, ina219_power_mW, ina219_loadvoltage);
}

void Rover6::setup_motors()
{
    pinMode(MOTOR_STBY, OUTPUT);
    motorA->begin();
    motorB->begin();
    print_info("Motors initialized.");
}

void Rover6::set_motor_standby(bool standby)
{
    if (standby) {  // set motors to low power
        digitalWrite(MOTOR_STBY, LOW);
    }
    else {  // bring motors out of standby mode
        digitalWrite(MOTOR_STBY, HIGH);
    }
}

void Rover6::setup_encoders()
{
    motorA_enc = new Encoder(MOTORA_ENCA, MOTORA_ENCB);
    motorB_enc = new Encoder(MOTORB_ENCA, MOTORB_ENCB);
    print_info("Encoders initialized.");
}

void Rover6::read_encoders()
{
    encA_pos = motorA_enc->read();
    encB_pos = motorB_enc->read();
}

void Rover6::report_encoders() {
    write("enc", "lll", millis(), encA_pos, encB_pos);
}

void Rover6::setup_VL53L0X()
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
    if(!lox1->begin(LOX1_ADDRESS, false, &Wire)) {
        print_error(F("Failed to boot first VL53L0X"));
        while(1);
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if(!lox2->begin(LOX2_ADDRESS, false, &Wire)) {
        print_error(F("Failed to boot second VL53L0X"));
        while(1);
    }
    print_info("VL53L0X's initialized.");
}
void Rover6::read_VL53L0X()
{
    lox1->rangingTest(measure1, false); // pass in 'true' to get debug data printout!
    lox2->rangingTest(measure2, false); // pass in 'true' to get debug data printout!

    // lox1_out_of_range = measure1->RangeStatus == 4;  // if out of range
    // measure1.RangeMilliMeter
}

void Rover6::report_VL53L0X() {
    write("lox", "lddd", millis(), measure1->RangeMilliMeter, measure2->RangeMilliMeter, measure1->RangeStatus);
}

void Rover6::setup_fsrs()
{
    pinMode(FSR_PIN_1, INPUT);
    pinMode(FSR_PIN_2, INPUT);
    print_info("FSRs initialized.");
}

void Rover6::read_fsrs()
{
    fsr_1_val = analogRead(FSR_PIN_1);
    fsr_2_val = analogRead(FSR_PIN_2);
}

void Rover6::report_fsrs() {
    write("fsr", "ldd", millis(), fsr_1_val, fsr_2_val);
}

void Rover6::initialize_display()
{
    pinMode(TFT_LITE, OUTPUT);
    tft->initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    delay(10);
    set_display_brightness(255);
    tft->fillScreen(ST77XX_BLACK);
    print_info("TFT display initialized.");
}

void Rover6::set_display_brightness(int brightness)
{
    analogWrite(TFT_LITE, brightness);
}

void Rover6::display_image(String encoded_image)
{
    /*
     * Display is 128x64 16-bit color
     * String must be at least length 16384 (0x4000)
     */

    if (encoded_image.length() < 0x4000) {
        print_error("Invalid string length");
        print_error(encoded_image.length());
        return;
    }
    uint8_t row;
    uint8_t col;
    for (size_t i = 0; i < encoded_image.length(); i += 2) {
        row = (uint8_t)(i / tft->height());
        col = i % tft->width();

        uint8_t c1 = (uint8_t)encoded_image.charAt(i);
        uint8_t c2 = (uint8_t)encoded_image.charAt(i + 1);
        uint16_t color = c1 << 8 | c2;
        tft.drawPixel(col, row, color);
    }
}

void Rover6::setup_BNO055()
{
    if (!bno->begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        print_error("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    print_info("BNO055 initialized.");
    delay(500);
}

void Rover6::read_BNO055()
{
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    bno->getEvent(orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno->getEvent(linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void Rover6::report_BNO055()
{
    write(
        "bno", "lfffffffff",
        orientationData->orientation.x,
        orientationData->orientation.y,
        orientationData->orientation.z,
        angVelocityData->gyro.x,
        angVelocityData->gyro.y,
        angVelocityData->gyro.z,
        linearAccelData->acceleration.x,
        linearAccelData->acceleration.y,
        linearAccelData->acceleration.z
    );
}


void Rover6::setup_IR()
{
    irrecv->enableIRIn();
    irrecv->blink13(false);
}

void Rover6::read_IR()
{
    if (irrecv->decode(irresults)) {
        ir_result_available = true;
        irrecv->resume(); // Receive the next value
    }
}


void Rover6::report_IR()
{
    if (!ir_result_available) {
        return;
    }
    ir_result_available = false;

    write("irr", "dd", irresults->decode_type, irresults->value);
}

void Rover6::apply_IR()
{
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
    if (irresults->decode_type == NEC) {
        switch (irresults->value) {
            case 0: break;
            case 1: break;
            case 2: break;
            case 4: break;
            case 5: break;
            case 6: break;
            case 8: break;
            case 9: break;
            case 10: break;
            case 12: break;
            case 13: break;
            case 14: break;
            case 16: break;
            case 17: break;
            case 18: break;
            case 20: break;
            case 21: break;
            case 22: break;
            case 24: break;
            case 25: break;
            case 26: break;
        }
    }
}
