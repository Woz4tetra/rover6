
// uint32_t current_time = 0;
#define CURRENT_TIME millis()

#ifndef ROVER6_GENERAL
#define ROVER6_GENERAL

#include <Arduino.h>
#include <Snooze.h>

#include "rover6_serial.h"

/*
 * Soft restart
 */
//
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

void soft_restart()
{
    MSG_SERIAL.end();
    DATA_SERIAL.end();
    SCB_AIRCR = 0x05FA0004;  // write value for restart
}

// low power mode
SnoozeDigital digital; // digital wake up driver
SnoozeBlock config(digital);
void enter_deep_sleep() {
    Snooze.deepSleep(config);
}


// Safety systems
struct safety {
    bool is_left_bumper_trig;
    bool is_right_bumper_trig;
    bool is_front_tof_trig;
    bool is_back_tof_trig;
    bool is_front_tof_ok;
    bool is_back_tof_ok;
    bool are_servos_active;
    bool are_motors_active;
    bool voltage_ok;
} safety_struct;

struct state {
    bool is_active;
    bool is_reporting_enabled;
    bool is_speed_pid_enabled;
} rover_state;

void init_structs() {
    safety_struct.is_left_bumper_trig = false;
    safety_struct.is_right_bumper_trig = false;
    safety_struct.is_front_tof_trig = false;
    safety_struct.is_back_tof_trig = false;
    safety_struct.is_front_tof_ok = false;
    safety_struct.is_back_tof_ok = false;
    safety_struct.are_servos_active = false;
    safety_struct.are_motors_active = false;
    safety_struct.voltage_ok = false;

    rover_state.is_active = false;
    rover_state.is_reporting_enabled = false;
    rover_state.is_speed_pid_enabled = false;
}

bool is_safe_to_move() {
    return safety_struct.voltage_ok && safety_struct.are_servos_active && safety_struct.are_motors_active && safety_struct.is_front_tof_ok && safety_struct.is_back_tof_ok;
}

bool is_obstacle_in_front() {
    return safety_struct.is_left_bumper_trig || safety_struct.is_right_bumper_trig || safety_struct.is_front_tof_trig;
}

bool is_obstacle_in_back() {
    return safety_struct.is_back_tof_trig;
}

void report_structs() {
    print_data(5, "ldddddddddddd", CURRENT_TIME,
        safety_struct.is_left_bumper_trig,
        safety_struct.is_right_bumper_trig,
        safety_struct.is_front_tof_trig,
        safety_struct.is_back_tof_trig,
        safety_struct.is_front_tof_ok,
        safety_struct.is_back_tof_ok,
        safety_struct.are_servos_active,
        safety_struct.are_motors_active,
        safety_struct.voltage_ok,
        rover_state.is_active,
        rover_state.is_reporting_enabled,
        rover_state.is_speed_pid_enabled
    );
}

// RPi connection
uint32_t prev_date_str_update = 0;
String rpi_date_str = "12:00:00AM";
bool broadcasting_hotspot = false;

#endif // ROVER6_GENERAL
