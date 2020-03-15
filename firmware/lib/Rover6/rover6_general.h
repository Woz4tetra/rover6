
#ifndef ROVER6_GENERAL
#define ROVER6_GENERAL

#include <Arduino.h>
#include "rover6_serial.h"

// uint32_t current_time = 0;
#define CURRENT_TIME millis()


/*
 * Soft restart
 */
//
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

namespace rover6
{
    void soft_restart()
    {
        DATA_SERIAL.end();  // clears the serial monitor  if used
        SCB_AIRCR = 0x05FA0004;  // write value for restart
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

    struct rpi_state {
        String ip_address;
        String hostname;
        String date_str;
        bool power_button_state;
        uint32_t prev_date_str_update;
        int broadcasting_hotspot;
    } rover_rpi_state;

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

        rover_rpi_state.ip_address = "";
        rover_rpi_state.hostname = "";
        rover_rpi_state.date_str = "12:00:00AM";
        rover_rpi_state.power_button_state = false;
        rover_rpi_state.prev_date_str_update = 0;
        rover_rpi_state.broadcasting_hotspot = 0;  // 0 == unknown, 1 == connected to wifi, 2 == broadcasting hotspot
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
        rover6_serial::data->write("safe", "ldddddddddddd", CURRENT_TIME,
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
        rover6_serial::info->write(rover6_serial::data->get_written_packet());
    }
};  // namespace rover6

#endif // ROVER6_GENERAL
