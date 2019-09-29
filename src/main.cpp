
#include <i2c_t3.h>
#include <Adafruit_INA219_Teensy.h>
#include <Adafruit_PWMServoDriverTeensy.h>
#include <Adafruit_VL53L0X_Teensy.h>

#include <TB6612.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include <Adafruit_NeoPixel.h>

#include <Encoder.h>

#include <Adafruit_BNO055_Teensy.h>


// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, &Wire1);

Adafruit_INA219 ina219;

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

TB6612 motorA(MOTORA_PWM, MOTORA_DR1, MOTORA_DR2);
TB6612 motorB(MOTORB_PWM, MOTORB_DR1, MOTORB_DR2);

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 5

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_LITE   6
float TFT_PI = 3.1415926;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// #define NEOPIXEL_RING_PIN  6
// #define NEOPIXEL_NUM_LEDS  24
// #define BRIGHTNESS  255
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM_LEDS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);


#define FSR_PIN_1 35
#define FSR_PIN_2 36


Encoder motorA_enc(MOTORA_ENCA, MOTORA_ENCB);
Encoder motorB_enc(MOTORB_ENCA, MOTORB_ENCB);

long motorA_enc_pos = 0;
long motorB_enc_pos = 0;


#define BNO055_RST_PIN 25
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire1);



void blink_builtin()
{
    for (size_t i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}


void setL0XID() {
    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // initing LOX1
    if(!lox1.begin(LOX1_ADDRESS, false, &Wire)) {
        Serial.println(F("Failed to boot first VL53L0X"));
        while(1);
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if(!lox2.begin(LOX2_ADDRESS, false, &Wire)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while(1);
    }
}


void read_dual_sensors() {

    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

    // print sensor one reading
    Serial.print("1: ");
    if(measure1.RangeStatus != 4) {     // if not out of range
        Serial.print(measure1.RangeMilliMeter);
    } else {
        Serial.print("Out of range");
    }

    Serial.print(" ");

    // print sensor two reading
    Serial.print("2: ");
    if(measure2.RangeStatus != 4) {
        Serial.print(measure2.RangeMilliMeter);
    } else {
        Serial.print("Out of range");
    }

    Serial.println();
}


void testlines(uint16_t color) {
    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawLine(0, 0, x, tft.height()-1, color);
        delay(0);
    }
    for (int16_t y=0; y < tft.height(); y+=6) {
        tft.drawLine(0, 0, tft.width()-1, y, color);
        delay(0);
    }

    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
        delay(0);
    }
    for (int16_t y=0; y < tft.height(); y+=6) {
        tft.drawLine(tft.width()-1, 0, 0, y, color);
        delay(0);
    }

    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawLine(0, tft.height()-1, x, 0, color);
        delay(0);
    }
    for (int16_t y=0; y < tft.height(); y+=6) {
        tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
        delay(0);
    }

    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
        delay(0);
    }
    for (int16_t y=0; y < tft.height(); y+=6) {
        tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
        delay(0);
    }
}

void testdrawtext(char *text, uint16_t color) {
    tft.setCursor(0, 0);
    tft.setTextColor(color);
    tft.setTextWrap(true);
    tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
    tft.fillScreen(ST77XX_BLACK);
    for (int16_t y=0; y < tft.height(); y+=5) {
        tft.drawFastHLine(0, y, tft.width(), color1);
    }
    for (int16_t x=0; x < tft.width(); x+=5) {
        tft.drawFastVLine(x, 0, tft.height(), color2);
    }
}

void testdrawrects(uint16_t color) {
    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
    }
}

void testfillrects(uint16_t color1, uint16_t color2) {
    tft.fillScreen(ST77XX_BLACK);
    for (int16_t x=tft.width()-1; x > 6; x-=6) {
        tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
        tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
    }
}

void testfillcircles(uint8_t radius, uint16_t color) {
    for (int16_t x=radius; x < tft.width(); x+=radius*2) {
        for (int16_t y=radius; y < tft.height(); y+=radius*2) {
            tft.fillCircle(x, y, radius, color);
        }
    }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
    for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
        for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
            tft.drawCircle(x, y, radius, color);
        }
    }
}

void testtriangles() {
    tft.fillScreen(ST77XX_BLACK);
    int color = 0xF800;
    int t;
    int w = tft.width()/2;
    int x = tft.height()-1;
    int y = 0;
    int z = tft.width();
    for(t = 0 ; t <= 15; t++) {
        tft.drawTriangle(w, y, y, x, z, x, color);
        x-=4;
        y+=4;
        z-=4;
        color+=100;
    }
}

void testroundrects() {
    tft.fillScreen(ST77XX_BLACK);
    int color = 100;
    int i;
    int t;
    for(t = 0 ; t <= 4; t+=1) {
        int x = 0;
        int y = 0;
        int w = tft.width()-2;
        int h = tft.height()-2;
        for(i = 0 ; i <= 16; i+=1) {
            tft.drawRoundRect(x, y, w, h, 5, color);
            x+=2;
            y+=3;
            w-=4;
            h-=6;
            color+=1100;
        }
        color+=100;
    }
}

void tftPrintTest() {
    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 30);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(1);
    tft.println("Hello World!");
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.println("Hello World!");
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(3);
    tft.println("Hello World!");
    tft.setTextColor(ST77XX_BLUE);
    tft.setTextSize(4);
    tft.print(1234.567);
    delay(1500);
    tft.setCursor(0, 0);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(0);
    tft.println("Hello World!");
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(TFT_PI, 6);
    tft.println(" Want pi?");
    tft.println(" ");
    tft.print(8675309, HEX); // print 8,675,309 out in HEX!
    tft.println(" Print HEX!");
    tft.println(" ");
    tft.setTextColor(ST77XX_WHITE);
    tft.println("Sketch has been");
    tft.println("running for: ");
    tft.setTextColor(ST77XX_MAGENTA);
    tft.print(millis() / 1000);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" seconds.");
}

void mediabuttons() {
    // play
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
    tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
    delay(500);
    // pause
    tft.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
    tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
    tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
    delay(500);
    // play color
    tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_BLUE);
    delay(50);
    // pause color
    tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_RED);
    tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_RED);
    // play color
    tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_GREEN);
}


void set_tft_light(uint8_t brightness) {
    analogWrite(TFT_LITE, brightness);
}


void displayDemo()
{
    uint16_t time = millis();
    tft.fillScreen(ST77XX_BLACK);
    time = millis() - time;

    Serial.println(time, DEC);
    delay(500);

    // large block of text
    // tft.fillScreen(ST77XX_BLACK);
    // testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
    // delay(1000);

    // tft print function!
    // tftPrintTest();
    // delay(4000);

    // a single pixel
    // tft.drawPixel(tft.width()/2, tft.height()/2, ST77XX_GREEN);
    // delay(500);

    // line draw test
    testlines(ST77XX_YELLOW);
    delay(500);

    // optimized lines
    testfastlines(ST77XX_RED, ST77XX_BLUE);
    delay(500);

    testdrawrects(ST77XX_GREEN);
    delay(500);

    testfillrects(ST77XX_YELLOW, ST77XX_MAGENTA);
    delay(500);

    tft.fillScreen(ST77XX_BLACK);
    testfillcircles(10, ST77XX_BLUE);
    testdrawcircles(10, ST77XX_WHITE);
    delay(500);

    testroundrects();
    delay(500);

    testtriangles();
    delay(500);

    for (int i = 255; i <= 0; i--) {
        set_tft_light(i);
        delay(5);
    }

    for (size_t i = 0; i < 256; i++) {
        set_tft_light(i);
        delay(5);
    }

    // mediabuttons();
    // delay(500);

    Serial.println("done");
    delay(1000);

    // tft.invertDisplay(true);
    // delay(500);
    // tft.invertDisplay(false);
    // delay(3500);
}

void setup() {
    pinMode(TFT_LITE, OUTPUT);
    for (size_t i = 0; i < 3; i++) {
        set_tft_light(0);
        delay(500);
        set_tft_light(255);
        delay(500);
    }

    // Use this initializer if using a 1.8" TFT screen:
    tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    delay(10);
    tft.fillScreen(ST77XX_BLACK);

    Serial.begin(115200);
    while (!Serial) {
        delay(1);
    }

    Serial.println("Rover #6 test!");

    Serial5.begin(115200);

    // blink_builtin();


    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000); // 200ms
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
    Wire1.setDefaultTimeout(200000); // 200ms

    ina219.begin(&Wire);

    pwm.begin();

    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

    delay(10);

    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);
    motorA.begin();
    motorB.begin();

    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    Serial.println("Shutdown pins inited...");

    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);

    Serial.println("Both in reset mode...(pins are low)");


    Serial.println("Starting...");
    setL0XID();

    // strip.setBrightness(BRIGHTNESS);
    // strip.begin();
    // strip.show();
    // strip.clear();

    pinMode(FSR_PIN_1, INPUT);
    pinMode(FSR_PIN_2, INPUT);

    // pinMode(LED_BUILTIN, OUTPUT);
    // blink_builtin();

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    Serial.println("BNO055 initialized.");
    delay(1000);
}

void printEvent(sensors_event_t* event) {
    Serial.println();
    Serial.print(event->type);
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER || event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }

    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
}



void report_bno055()
{
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData , angVelocityData , linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);

    int8_t boardTemp = bno.getTemp();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);


    delay(BNO055_SAMPLERATE_DELAY_MS);
}

void report_ina219()
{
    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");
}

void setServoAngle(uint8_t n, double angle) {
    uint16_t pulse = (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
    Serial.println(pulse);
    pwm.setPWM(n, 0, pulse);
}

/*uint32_t char_to_color(char color_char) {
switch (color_char) {
case 'w': return strip.Color(255, 255, 255);
case 'b': return strip.Color(0, 0, 255);
case 'r': return strip.Color(255, 0, 0);
case 'g': return strip.Color(0, 255, 0);
case 'y': return strip.Color(255, 255, 0);
case 'k': return strip.Color(0, 0, 0);
default: return strip.Color(0, 0, 0);
}
}*/

int selected_servo = 0;
int servo_pos = 0;
int motor_speed = 0;
int led_demo_state = 0;
uint32_t current_color = 0;


void loop() {
    if (Serial5.available()) {
        Serial.println(Serial5.readStringUntil('\n'));
    }
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        Serial.println(command);
        switch (command.charAt(0)) {
            case 'i': report_ina219(); break;
            case 'd':
                for (size_t i = 0; i < 100; i++) {
                    read_dual_sensors();
                    delay(5);
                }
                break;
            case 'm':
                motor_speed = command.substring(2).toInt();
                if (command.charAt(1) == 'a') {
                    motorA.setSpeed(motor_speed);
                    Serial.println("Motor A");
                }
                else if (command.charAt(1) == 'b') {
                    motorB.setSpeed(motor_speed);
                    Serial.println("Motor B");
                }
                Serial.print("motor_speed:\t");
                Serial.println(motor_speed);
                break;
            case 's':
                selected_servo = (int)(command.charAt(1) - '0');
                servo_pos = command.substring(2).toInt();
                Serial.print("selected_servo:\t");
                Serial.println(selected_servo);
                Serial.print("servo_pos:\t");
                Serial.println(servo_pos);
                setServoAngle(selected_servo, servo_pos);
                break;
            case 'x': displayDemo(); break;
            /*case 'n':
                // can't do fancy animations since logic is 3.3V and supply
                // is 5V. We'll let the TFT screen do all the fancy stuff
                current_color = char_to_color(command.charAt(1));
                // if (led_demo_state == 0) {
                //     current_color = strip.Color(50, 50, 50);
                //     led_demo_state = 1;
                // }
                // else {
                //     current_color = strip.Color(0, 0, 0);
                //     led_demo_state = 0;
                // }
                Serial.print("Setting color to:\t");
                Serial.println(current_color);
                for(int i = 0; i <= NEOPIXEL_NUM_LEDS; i++) {
                    strip.setPixelColor(i, current_color);
                }
                strip.show();

                Serial.println("Neopixel demo done.");
                break;*/
            case 'f':
                for (size_t i = 0; i < 100; i++) {
                    Serial.print("1: ");
                    Serial.print(analogRead(FSR_PIN_1));
                    Serial.print("\t2: ");
                    Serial.println(analogRead(FSR_PIN_2));
                    delay(50);
                }

                break;
            case 'e':
                motorA_enc_pos = motorA_enc.read();
                motorB_enc_pos = motorB_enc.read();
                Serial.print("A: ");
                Serial.print(motorA_enc_pos);
                Serial.print("\tB: ");
                Serial.println(motorB_enc_pos);
                break;
            case '5':
                Serial5.println(command.substring(1));
                break;
            case 'b':
                for (size_t i = 0; i < 100; i++) {
                    report_bno055();
                }
                break;
        }
    }
}
