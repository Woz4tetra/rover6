#ifndef ROVER6_PID
#define ROVER6_PID


#include <rover6_general.h>
#include <rover6_serial.h>
#include <rover6_encoders.h>
#include <rover6_motors.h>

#include <PID_v1.h>

/*
 * Motor speed controller
 */
//

namespace rover6_pid
{
    double speed_setpointA, speed_setpointB = 0.0;  // cm/s
    double pid_commandA, pid_commandB = 0.0;  // -255...255
    double Kp_A = 0.01, Ki_A = 0.0, Kd_A = 0.0;
    double Kp_B = 0.01, Ki_B = 0.0, Kd_B = 0.0;
    #define NUM_PID_KS 8
    double* pid_Ks = new double[NUM_PID_KS];
    uint32_t prev_setpointA_time = 0;
    uint32_t prev_setpointB_time = 0;
    #define PID_COMMAND_TIMEOUT_MS 1000
    #define PID_UPDATE_DELAY_MS 33  // ~30 Hz
    uint32_t prev_pid_time = 0;
    double ff_command_A, ff_command_B = 0;  // feed forward commands
    double ff_speed_A, ff_speed_B = 0;  // feed forward speed. enc_speed - setpoint
    double ff_setpoint_A, ff_setpoint_B = 0.0;  // always zero
    const unsigned int min_usable_commmand = 35;

    // 160 rpm @ 6V
    // 135 rpm @ 5V
    // 60 rpm @ 3V
    // double max_rpm = 135.0;
    // double max_linear_speed_cps = max_rpm * 2.0 * PI * wheel_radius_cm / 60.0;  // cm per s, no load
    // double max_linear_speed_cps = 915.0;
    // double cps_to_cmd = 255.0 / max_linear_speed_cps;
    double max_linear_speed_tps = 8200.0;  // max speed measured in ticks per second (~915cm/s)
    double tps_to_cmd = 255.0 / max_linear_speed_tps;

    PID motorA_pid(&ff_speed_A, &pid_commandA, &ff_setpoint_A, Kp_A, Ki_A, Kd_A, DIRECT);
    PID motorB_pid(&ff_speed_B, &pid_commandB, &ff_setpoint_B, Kp_B, Ki_B, Kd_B, DIRECT);

    void set_Ks()
    {
        Kp_A = pid_Ks[0];
        Ki_A = pid_Ks[1];
        Kd_A = pid_Ks[2];
        Kp_B = pid_Ks[3];
        Ki_B = pid_Ks[4];
        Kd_B = pid_Ks[5];
        rover6_encoders::speed_smooth_kA = pid_Ks[6];  // defined in rover6_encoders.h
        rover6_encoders::speed_smooth_kB = pid_Ks[7];  // defined in rover6_encoders.h
        motorA_pid.SetTunings(Kp_A, Ki_A, Kd_A);
        motorB_pid.SetTunings(Kp_B, Ki_B, Kd_B);
    }

    void setup_pid()
    {
        motorA_pid.SetMode(AUTOMATIC);
        motorB_pid.SetMode(AUTOMATIC);

        for (size_t index = 0; index < NUM_PID_KS; index++){
            pid_Ks[index] = 0.0;
        }
        pid_Ks[0] = Kp_A;
        pid_Ks[1] = Ki_A;
        pid_Ks[2] = Kd_A;
        pid_Ks[3] = Kp_B;
        pid_Ks[4] = Ki_B;
        pid_Ks[5] = Kd_B;
    }

    void update_setpointA(double new_setpoint)
    {
        speed_setpointA = new_setpoint;
        ff_command_A = speed_setpointA * tps_to_cmd;
        prev_setpointA_time = CURRENT_TIME;
        // print_info("speed_setpointA: ");
        // DATA_SERIAL.println(speed_setpointA);
    }

    void update_setpointB(double new_setpoint)
    {
        speed_setpointB = new_setpoint;
        ff_command_B = speed_setpointB * tps_to_cmd;
        prev_setpointB_time = CURRENT_TIME;
        // print_info("speed_setpointB: ");
        // DATA_SERIAL.println(speed_setpointB);
    }

    void update_speed_pid()
    {
        if (!rover6::rover_state.is_speed_pid_enabled) {
            return;
        }

        if (CURRENT_TIME - prev_pid_time < PID_UPDATE_DELAY_MS) {
            return;
        }
        prev_pid_time = CURRENT_TIME;
        ff_speed_A = rover6_encoders::enc_speedA - speed_setpointA;
        ff_speed_B = rover6_encoders::enc_speedB - speed_setpointB;

        if (speed_setpointA != 0.0) {
            if (CURRENT_TIME - prev_setpointA_time > PID_COMMAND_TIMEOUT_MS) {
                rover6_serial::println_info("PID A setpoint timed out");
                update_setpointA(0.0);
            }
        }

        if (speed_setpointB != 0.0) {
            if (CURRENT_TIME - prev_setpointB_time > PID_COMMAND_TIMEOUT_MS) {
                rover6_serial::println_info("PID B setpoint timed out");
                update_setpointB(0.0);
            }
        }

        // inputs: ff_speed (measured - setpoint), ff_setpoint (always 0)
        // output: pid_command (-255..255)
        motorA_pid.Compute();
        motorB_pid.Compute();
        // pid_commandA = 0.0;
        // pid_commandB = 0.0;

        // print_info("ff_speed_A: ");
        // DATA_SERIAL.println(ff_speed_A);
        // print_info("ff_speed_B: ");
        // DATA_SERIAL.println(ff_speed_B);
        // print_info("pid_commandA: ");
        // DATA_SERIAL.println(pid_commandA);
        // print_info("pid_commandB: ");
        // DATA_SERIAL.println(pid_commandB);

        // apply feed forward commands
        int sum_commandA = pid_commandA + ff_command_A;
        int sum_commandB = pid_commandB + ff_command_B;
        if (abs(sum_commandA) < min_usable_commmand) {
            sum_commandA = 0;
        }
        if (abs(sum_commandB) < min_usable_commmand) {
            sum_commandB = 0;
        }

        rover6_motors::set_motors(sum_commandA, sum_commandB);
    }

    void set_speed_pid(bool enabled) {
        rover6::rover_state.is_speed_pid_enabled = enabled;
    }
};  // namespace rover6_pid

#endif  // ROVER6_PID
