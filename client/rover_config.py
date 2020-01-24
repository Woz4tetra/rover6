import math

class RoverConfig:
    wheel_radius_cm = 32.5
    cm_per_tick = 2.0 * math.pi * wheel_radius_cm / 1920.0
    max_linear_speed_cps = 915.0
    cps_to_cmd = 255.0 / max_linear_speed_cps
    tof_off_axis_mm = 15.0  # how far the sensor is from the axis of rotation
    tof_ground_dist_mm = 28.7  # how far the axis of rotation is off the ground 
    tof_front_wall_dist_mm = 30.8  # how far the front wall is from the axis of rotation
    tof_back_wall_dist_mm = 15.0  # how far the back wall is from the axis of rotation

    kp = 1.0
    ki = 0.0
    kd = 0.0

    front_ledge_y_mm = 100
    front_obstacle_x_mm = 100
    back_ledge_y_mm = 100
    back_obstacle_x_mm = 100

    tof_servo_upper_command = 90
    tof_servo_lower_command = 180
    tof_servo_upper_angle_deg = 360.0
    tof_servo_lower_angle_deg = 275.0

    front_tilter_servo_num = 0
    back_tilter_servo_num = 1

    camera_pan_servo_num = 2
    camera_tilt_servo_num = 3
