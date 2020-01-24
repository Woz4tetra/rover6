import time
import select
import numpy as np

from evdev import ecodes, InputDevice

from rover_client import RoverClient, RoverConfig
import tof_obstacles


def open_joystick():
    try:
        return InputDevice("/dev/input/event0")
    except FileNotFoundError:
        return None

max_speed = 915.0
max_joy_val = 32767 # 0x7fff
deadzone_joy_val = 8000
def joy_to_speed(value):
    joy_val = max_joy_val - value
    if abs(joy_val) < deadzone_joy_val:
        return 0
    
    if joy_val >= 0:
        speed_cps = max_speed / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val)
    else:
        joy_val = abs(joy_val)
        speed_cps = -max_speed / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val)

    return speed_cps


def joy_to_pan_servo(value):
    joy_val = max_joy_val - value
    if abs(joy_val) < deadzone_joy_val:
        return None
    if joy_val >= 0:
        command = (RoverConfig.pan_right_command - RoverConfig.pan_left_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + RoverConfig.pan_center_command
    else:
        joy_val = abs(joy_val)
        command = -(RoverConfig.pan_right_command - RoverConfig.pan_left_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + RoverConfig.pan_center_command
    
    return int(command)


def joy_to_tilt_servo(value):
    joy_val = max_joy_val - value
    if abs(joy_val) < deadzone_joy_val:
        return None
    if joy_val >= 0:
        command = (RoverConfig.tilt_up_command - RoverConfig.tilt_down_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + RoverConfig.tilt_center_command
    else:
        joy_val = abs(joy_val)
        command = -(RoverConfig.tilt_up_command - RoverConfig.tilt_down_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + RoverConfig.tilt_center_command
    
    return int(command)


def main():    
    rover = RoverClient()

    forward_speed = 0.0
    rotate_speed = 0.0
    speed_A = 0.0
    speed_B = 0.0
    pan_servo = None
    tilt_servo = None
    device = None

    rover.start()

    prev_open_attempt_time = time.time()
    try:
        rover.set_k(0.40, 0.0, 0.01)
        rover.set_safety_thresholds(110, 35, 20)
        # rover.set_obstacle_thresholds(120, 0x10000, 0)
        # rover.set_servo(RoverConfig.front_tilter_servo_num, 180)
        # rover.set_obstacle_thresholds(120, 0x10000, 1)
        # rover.set_servo(RoverConfig.back_tilter_servo_num, 180)


        while True:
            if not device:
                if time.time() - prev_open_attempt_time > 1:
                    device = open_joystick()
                    prev_open_attempt_time = time.time()
                    if device:
                        print("Joystick opened")
                else:
                    time.sleep(1)
            else:
                select.select([device.fd], [], [], 0.01)

                try:
                    for event in device.read():
                        codename = process_event(event)
                        if codename == "ABS_Y":
                            forward_speed = joy_to_speed(event.value)
                        elif codename == "ABS_X":
                            rotate_speed = joy_to_speed(event.value)
                        elif codename == "ABS_Z":
                            pan_servo = joy_to_pan_servo(event.value)
                        elif codename == "ABS_RZ":
                            tilt_servo = joy_to_tilt_servo(event.value)
                    speed_A = forward_speed + rotate_speed
                    speed_B = forward_speed - rotate_speed
                except BlockingIOError:
                    continue
                
                rover.set_speed(speed_A, speed_B)
                rover.set_servo(RoverConfig.pan_servo_num, pan_servo)
                rover.set_servo(RoverConfig.tilt_servo_num, tilt_servo)
                time.sleep(1/30)

    except BaseException:
        raise
    finally:
        rover.stop()
        print(rover.data_frame)
        for identifier, times in rover.recv_times.items():
            print("%s:\t%sHz" % (identifier, np.mean(1 / np.diff(times))))

def process_event(e):
    if e.type == ecodes.EV_SYN:
        if e.code == ecodes.SYN_MT_REPORT:
            msg = 'time {:<16} +++++++++ {} ++++++++'
        else:
            msg = 'time {:<16} --------- {} --------'
        # print(msg.format(e.timestamp(), ecodes.SYN[e.code]))
        return None
    else:
        if e.type in ecodes.bytype:
            codename = ecodes.bytype[e.type][e.code]
        else:
            codename = '?'

        # evfmt = 'time {:<16} type {} ({}), code {:<4} ({}), value {}'
        # print(evfmt.format(e.timestamp(), e.type, ecodes.EV[e.type], e.code, codename, e.value))

        return codename


if __name__ == "__main__":
    main()
