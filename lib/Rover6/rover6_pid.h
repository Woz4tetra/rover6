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

void set_Kp_A(double Kp)
{
    Kp_A = Kp;
    motorA_pid.SetTunings(Kp_A, Ki_A, Kd_A);
}

void set_Ki_A(double Ki)
{
    Ki_A = Ki;
    motorA_pid.SetTunings(Kp_A, Ki_A, Kd_A);
}

void set_Kd_A(double Kd)
{
    Kd_A = Kd;
    motorA_pid.SetTunings(Kp_A, Ki_A, Kd_A);
}


void set_Kp_B(double Kp)
{
    Kp_B = Kp;
    motorA_pid.SetTunings(Kp_B, Ki_B, Kd_B);
}

void set_Ki_B(double Ki)
{
    Ki_B = Ki;
    motorA_pid.SetTunings(Kp_B, Ki_B, Kd_B);
}

void set_Kd_B(double Kd)
{
    Kd_B = Kd;
    motorA_pid.SetTunings(Kp_B, Ki_B, Kd_B);
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
    // MSG_SERIAL.println(speed_setpointA);
}

void update_setpointB(double new_setpoint)
{
    speed_setpointB = new_setpoint;
    ff_command_B = speed_setpointB * cps_to_cmd;
    prev_setpointB_time = CURRENT_TIME;
    // print_info("speed_setpointB: ");
    // MSG_SERIAL.println(speed_setpointB);
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
    // MSG_SERIAL.println(ff_speed_A);
    // print_info("ff_speed_B: ");
    // MSG_SERIAL.println(ff_speed_B);

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

#endif  // ROVER6_PID