import time
import select

from evdev import ecodes, InputDevice

from rover_client import RoverClient

rover = RoverClient()

speed_A = 0.0
speed_B = 0.0
max_speed = 915.0

def main():    
    global speed_A, speed_B
    device = InputDevice("/dev/input/event0")
    # device.grab()

    print('Listening for events (press ctrl-c to exit) ...')

    rover.start()

    try:
        while True:
            select.select([device.fd], [], [], 1/60)
            # speed_A = 0.0
            # speed_B = 0.0
            rover.set_speed(speed_A, speed_B)

            try:
                for event in device.read():
                    codename = process_event(event)
                    if codename == "ABS_Y":
                        joy_val = 0x7fff - event.value
                        if abs(joy_val) < 4000:
                            joy_val = 0
                        speed_B = (joy_val) * (max_speed / 0x7fff)
                    elif codename == "ABS_RZ":
                        joy_val = 0x7fff - event.value
                        if abs(joy_val) < 4000:
                            joy_val = 0
                        speed_A = (joy_val) * (max_speed / 0x7fff)
            except BlockingIOError:
                continue
            
            time.sleep(1/60)



    except BaseException:
        raise
    finally:
        rover.stop()

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
