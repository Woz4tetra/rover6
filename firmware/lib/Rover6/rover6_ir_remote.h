#ifndef ROVER6_IR_REMOTE
#define ROVER6_IR_REMOTE

#include <IRremote.h>

#include "rover6_serial.h"
#include "rover6_general.h"

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
        println_info("IR: %d", ir_value);
        return true;
    }
    else {
        return false;
    }
}
void report_IR()
{
    if (!rover_state.is_reporting_enabled) {
        return;
    }
    print_data("ir", "udd", CURRENT_TIME, ir_type, ir_value);
}

#endif  // ROVER6_IR_REMOTE
