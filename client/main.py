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
max_joy_val = 0x7fff
deadzone_joy_val = 0x3fff
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


def main():    
    rover = RoverClient()

    forward_speed = 0.0
    rotate_speed = 0.0
    speed_A = 0.0
    speed_B = 0.0
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
                select.select([device.fd], [], [], 1/60)

                try:
                    for event in device.read():
                        codename = process_event(event)
                        if codename == "ABS_Y":
                            forward_speed = joy_to_speed(event.value)
                        # elif codename == "ABS_RZ":
                        elif codename == "ABS_X":
                            rotate_speed = joy_to_speed(event.value)
                    speed_A = forward_speed + rotate_speed
                    speed_B = forward_speed - rotate_speed
                except BlockingIOError:
                    continue
                    
                rover.set_speed(speed_A, speed_B)
                time.sleep(1/60)

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
