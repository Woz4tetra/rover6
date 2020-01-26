import time
import numpy as np


from lib.rover.rover_client import RoverClient
from lib.joystick import Joystick
from lib.config import ConfigManager

rover_config = ConfigManager.get_rover_config()


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
        command = (rover_config.pan_right_command - rover_config.pan_left_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + rover_config.pan_center_command
    else:
        joy_val = abs(joy_val)
        command = -(rover_config.pan_right_command - rover_config.pan_left_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + rover_config.pan_center_command
    
    return int(command)


def joy_to_tilt_servo(value):
    joy_val = max_joy_val - value
    if abs(joy_val) < deadzone_joy_val:
        return None
    if joy_val >= 0:
        command = (rover_config.tilt_up_command - rover_config.tilt_down_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + rover_config.tilt_center_command
    else:
        joy_val = abs(joy_val)
        command = -(rover_config.tilt_up_command - rover_config.tilt_down_command) / (max_joy_val - deadzone_joy_val) * (joy_val - deadzone_joy_val) + rover_config.tilt_center_command
    
    return int(command)


def main():    
    rover = RoverClient()
    joystick = Joystick("/dev/input/event0")

    forward_speed = 0.0
    rotate_speed = 0.0
    pan_servo = None
    tilt_servo = None

    rover.start()
    
    try:
        rover.set_k(0.40, 0.0, 0.01)
        rover.set_safety_thresholds(110, 35, 20)
        # rover.set_obstacle_thresholds(120, 0x10000, 0)
        # rover.set_servo(rover_config.front_tilter_servo_num, 180)
        # rover.set_obstacle_thresholds(120, 0x10000, 1)
        # rover.set_servo(rover_config.back_tilter_servo_num, 180)

        while True:
            if not joystick.is_open():
                time.sleep(1.0)
            for result in joystick.get_events():
                if result is None:
                    continue
                codename, event = result
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
                
                rover.set_speed(speed_A, speed_B)
                rover.set_servo(rover_config.pan_servo_num, pan_servo)
                rover.set_servo(rover_config.tilt_servo_num, tilt_servo)
                
                time.sleep(1/30)

    except BaseException:
        raise
    finally:
        rover.stop()
        print(rover.data_frame)
        for identifier, times in rover.recv_times.items():
            print("%s:\t%sHz" % (identifier, np.mean(1 / np.diff(times))))


if __name__ == "__main__":
    main()
