#!/usr/bin/python
from __future__ import division
import os
import sys
import math
import datetime
import traceback

import tf
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

from rover6_chassis.cfg import Rover6ChassisConfig

from rover6_serial_bridge.msg import Rover6Encoder
from rover6_serial_bridge.msg import Rover6Motors

from rover6_serial_bridge.srv import Rover6PidSrv
from rover6_serial_bridge.srv import Rover6SafetySrv

print(sys.version)

class Rover6Chassis:
    def __init__(self):
        rospy.init_node(
            "rover6_chassis",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        # rospy.on_shutdown(self.shutdown)

        # robot dimensions
        self.wheel_radius_cm = rospy.get_param("~wheel_radius_cm", 3.25)
        self.wheel_distance_cm = rospy.get_param("~wheel_distance_cm", 17.0)
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 3840.0)
        self.motors_pub_name = rospy.get_param("~motors_pub_name", "motors")
        self.max_speed_cps = rospy.get_param("~max_speed_cps", 36.2)
        self.services_enabled = rospy.get_param("~services_enabled", True)
        self.use_sensor_msg_time = rospy.get_param("~use_sensor_msg_time", False)

        self.wheel_radius_m = self.wheel_radius_cm / 100.0
        self.wheel_distance_m = self.wheel_distance_cm / 100.0

        self.m_to_tick_factor = self.ticks_per_rotation / (2.0 * self.wheel_radius_m * math.pi)
        self.tick_to_m_factor = 1.0 / self.m_to_tick_factor

        self.max_speed_mps = self.max_speed_cps / 100.0
        self.max_speed_tps = self.max_speed_mps * self.m_to_tick_factor  # max speed in ticks per s

        # TF parameters
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.odom_parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.pan_tilt_frame = rospy.get_param("~pan_tilt_frame", "pan_tilt_link")
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Encoder variables
        self.enc_msg = Rover6Encoder()

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # motor message
        self.motors_msg = Rover6Motors()

        # Safety variables
        self.tof_servo_lower_command = rospy.get_param("~tof_servo_lower_command", 0)
        self.tof_servo_upper_command = rospy.get_param("~tof_servo_upper_command", 90)
        self.tof_servo_lower_angle_deg = rospy.get_param("~tof_servo_lower_angle_deg", 275.0)
        self.tof_servo_upper_angle_deg = rospy.get_param("~tof_servo_upper_angle_deg", 360.0)

        self.tof_ground_dist_mm = rospy.get_param("~tof_ground_dist_mm", 28.7)
        self.tof_off_axis_mm = rospy.get_param("~tof_off_axis_mm", 15.0)
        self.tof_front_wall_dist_mm = rospy.get_param("~tof_front_wall_dist_mm", 30.8)
        self.tof_back_wall_dist_mm = rospy.get_param("~tof_back_wall_dist_mm", 15.0)

        # pan tilt command limits
        self.pan_right_command = rospy.get_param("~pan_right_command", 90)
        self.pan_left_command = rospy.get_param("~pan_left_command", 0)
        self.pan_center_command = rospy.get_param("~pan_center_command", 45)
        self.pan_max_angle = math.radians(rospy.get_param("~pan_max_angle_deg", 45.0))
        self.pan_min_angle = math.radians(rospy.get_param("~pan_min_angle_deg", -45.0))

        self.tilt_up_command = rospy.get_param("~tilt_up_command", 0)
        self.tilt_down_command = rospy.get_param("~tilt_down_command", 150)
        self.tilt_center_command = rospy.get_param("~tilt_center_command", 100)
        self.tilt_max_angle = math.radians(rospy.get_param("~tilt_max_angle_deg", 90.0))
        self.tilt_min_angle = math.radians(rospy.get_param("~tilt_min_angle_deg", -45.0))

        # odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0

        # pan-tilt state
        self.servo_pan = 0.0
        self.servo_tilt = 0.0
        self.pan_tilt_xyz_tf = (0.0264, 0.0, 0.00241)

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_parent_frame
        self.odom_msg.child_frame_id = self.child_frame

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.encoder_sub = rospy.Subscriber("encoders", Rover6Encoder, self.encoder_callback, queue_size=100)

        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.motors_pub = rospy.Publisher(self.motors_pub_name, Rover6Motors, queue_size=100)

        # Services
        self.pid_service_name = "rover6_pid"
        self.set_pid = None

        self.safety_service_name = "rover6_safety"
        self.set_safety_thresholds = None

        if self.services_enabled:
            rospy.loginfo("Waiting for service %s" % self.pid_service_name)
            rospy.wait_for_service(self.pid_service_name)
            self.set_pid = rospy.ServiceProxy(self.pid_service_name, Rover6PidSrv)
            rospy.loginfo("%s service is ready" % self.pid_service_name)

            rospy.loginfo("Waiting for service %s" % self.safety_service_name)
            rospy.wait_for_service(self.safety_service_name)
            self.set_safety_thresholds = rospy.ServiceProxy(self.safety_service_name, Rover6SafetySrv)
            rospy.loginfo("%s service is ready" % self.safety_service_name)

            # dynamic reconfigure
            dyn_cfg = Server(Rover6ChassisConfig, lambda config, level: Rover6Chassis.dynamic_callback(self, config, level))

    def dynamic_callback(self, config, level):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled!")
            return

        if self.set_pid is None:
            rospy.logwarn("%s service not ready yet!" % self.pid_service_name)
        else:
            self.call_pid_service(config)

        if self.set_pid is None:
            rospy.logwarn("%s service not ready yet!" % self.safety_service_name)
        else:
            self.call_safety_service(config)
        return config

    def call_pid_service(self, config):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled!")
        try:
            self.set_pid(
                config["kp_A"],
                config["ki_A"],
                config["kd_A"],
                config["kp_B"],
                config["ki_B"],
                config["kd_B"],
                config["speed_kA"],
                config["speed_kB"],
            )
        except rospy.ServiceException, e:
            rospy.logwarn("%s service call failed: %s" % (self.pid_service_name, e))

    def call_safety_service(self, config):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled!")
        try:
            args = self.get_stopping_thresholds(
                config["obstacle_threshold_x_mm"],
                config["ledge_threshold_y_mm"],
                config["buffer_x_mm"],
                config["ranging_tilter_mode"],
            )
            self.set_safety_thresholds(*args)
        except rospy.ServiceException, e:
            rospy.logwarn("%s service call failed: %s" % (self.safety_service_name, e))

    def angle_rad_to_tof_servo_command(self, angle_rad):
        angle_rad = angle_rad + (math.pi * 2 if angle_rad <= 0.0 else 0)  # bound to 270...360 deg
        angle_deg = math.degrees(angle_rad)
        y0 = self.tof_servo_lower_command
        y1 = self.tof_servo_upper_command
        x0 = self.tof_servo_lower_angle_deg
        x1 = self.tof_servo_upper_angle_deg

        servo_command = (x1 - x0) / (y1 - y0) * (angle_deg - x0) + y0
        return int(servo_command)


    def calculate_tof_thresholds(self, obstacle_x, ledge_y, buffer_x, wall_offset):
        gaze_x = obstacle_x + buffer_x + wall_offset  # X coordinate where the sensor is pointing at
        threshold_x = obstacle_x + wall_offset  # X coordinate where threshold starts
        offset_ledge_y = ledge_y + self.tof_ground_dist_mm  # Y threshold relative to the axis of rotation
        sensor_angle = math.atan2(-self.tof_ground_dist_mm, gaze_x)

        # sensor lower threshold accounting for sensor's distance away from the axis of rotation
        obstacle_threshold = threshold_x / math.cos(sensor_angle) - self.tof_off_axis_mm

        # sensor upper threshold accounting for sensor's distance away from the axis of rotation
        ledge_threshold = abs(offset_ledge_y / math.sin(sensor_angle)) - self.tof_off_axis_mm

        servo_command = self.angle_rad_to_tof_servo_command(sensor_angle)
        assert obstacle_threshold >= 0.0, obstacle_threshold
        assert ledge_threshold >= 0.0, ledge_threshold
        return obstacle_threshold, ledge_threshold, servo_command


    def get_stopping_thresholds(self, obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm, ranging_tilter_mode):
        # obstacle_threshold_x_mm: measured from the maximum point of the front or back of the robot
        # ledge_threshold_y_mm: measured from the ground plane where the robot sits flat on the ground
        # buffer_x_mm: x buffer distance to account for robot stopping time and update rate delay

        front_obstacle_threshold, front_ledge_threshold, front_servo_command = \
            self.calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                     self.tof_front_wall_dist_mm)

        back_obstacle_threshold, back_ledge_threshold, back_servo_command = \
            self.calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                     self.tof_back_wall_dist_mm)


        # if ranging_tilter_mode == 0:  # look for both obstacles and ledges
        if ranging_tilter_mode == 1:  # look for obstacles only
            front_servo_command = 90
            back_servo_command = 90
            front_ledge_threshold = 0xffff
            back_ledge_threshold = 0xffff
        elif ranging_tilter_mode == 2:  # look for ledges only
            front_obstacle_threshold = 0
            back_obstacle_threshold = 0
        return (
            int(front_obstacle_threshold), int(front_ledge_threshold), int(front_servo_command),
            int(back_obstacle_threshold), int(back_ledge_threshold), int(back_servo_command)
        )

    def twist_callback(self, twist_msg):
        linear_speed_mps = twist_msg.linear.x  # m/s
        angular_speed_radps = twist_msg.angular.z  # rad/s

        # arc = angle * radius
        # rotation speed at the wheels
        rotational_speed_mps = angular_speed_radps * self.wheel_distance_m / 2

        left_command = self.m_to_ticks(linear_speed_mps - rotational_speed_mps)
        right_command = self.m_to_ticks(linear_speed_mps + rotational_speed_mps)

        self.motors_msg.left = left_command
        self.motors_msg.right = right_command

        self.motors_pub.publish(self.motors_msg)

    def servo_to_angle(self, command, max_command, min_command, max_angle, min_angle):
        return (max_angle - min_angle) / (max_command - min_command) * (command - min_command) + min_angle

    def servo_callback(self, servo_msg):
        self.servo_pan = servo_to_angle(servo_msg.camera_pan, self.pan_right_command, self.pan_left_command, self.pan_max_angle, self.pan_min_angle)
        self.servo_tilt = servo_to_angle(servo_msg.camera_tilt, self.tilt_down_command, self.tilt_up_command, self.tilt_max_angle, self.tilt_min_angle)

    def encoder_callback(self, enc_msg):
        self.enc_msg = enc_msg

    def run(self):
        clock_rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                self.compute_odometry()
                self.publish_chassis_data()
                self.publish_pan_tilt_tfs()
            except BaseException, e:
                traceback.print_exc()
                rospy.signal_shutdown(str(e))

            clock_rate.sleep()

    def publish_pan_tilt_tfs(self):
        now = rospy.Time.now()
        pan_tilt_quaternion = tf.transformations.quaternion_from_euler(0.0, self.servo_tilt, self.servo_pan)
        self.tf_broadcaster.sendTransform(
            self.pan_tilt_xyz_tf,
            pan_tilt_quaternion,
            now,
            self.child_frame,
            self.pan_tilt_frame
        )

    def publish_chassis_data(self):
        if self.use_sensor_msg_time:
            now = self.enc_msg.header.stamp
        else:
            now = rospy.Time.now()
        odom_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_t)
        self.tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.0),
            odom_quaternion,
            now,
            self.child_frame,
            self.odom_parent_frame
        )

        self.odom_msg.header.stamp = now
        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = odom_quaternion[0]
        self.odom_msg.pose.pose.orientation.y = odom_quaternion[1]
        self.odom_msg.pose.pose.orientation.z = odom_quaternion[2]
        self.odom_msg.pose.pose.orientation.w = odom_quaternion[3]

        self.odom_msg.twist.twist.linear.x = self.odom_vx
        self.odom_msg.twist.twist.linear.y = self.odom_vy
        self.odom_msg.twist.twist.linear.z = 0.0

        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = self.odom_vt

        self.odom_pub.publish(self.odom_msg)

    def ticks_to_m(self, ticks):
        return ticks * self.tick_to_m_factor

    def m_to_ticks(self, meters):
        return meters * self.m_to_tick_factor

    def compute_odometry(self):
        delta_left = self.ticks_to_m(self.enc_msg.left_ticks - self.prev_left_ticks)
        delta_right = self.ticks_to_m(self.enc_msg.right_ticks - self.prev_right_ticks)
        delta_dist = (delta_right + delta_left) / 2

        if abs(delta_dist) > 0.0001:
            left_speed = self.ticks_to_m(self.enc_msg.left_speed_ticks_per_s)
            right_speed = self.ticks_to_m(self.enc_msg.right_speed_ticks_per_s)
        else:
            left_speed = 0.0
            right_speed = 0.0

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / self.wheel_distance_m
        self.odom_t += delta_angle

        dx = delta_dist * math.cos(self.odom_t)
        dy = delta_dist * math.sin(self.odom_t)

        self.odom_x += dx
        self.odom_y += dy

        speed = (left_speed + right_speed) / 2
        self.odom_vx = speed * math.cos(self.odom_t)
        self.odom_vy = speed * math.sin(self.odom_t)
        self.odom_vt = (right_speed - left_speed) / (self.wheel_distance_m / 2)

        # print self.odom_x, self.odom_y, math.degrees(self.odom_t)

        self.prev_left_ticks = self.enc_msg.left_ticks
        self.prev_right_ticks = self.enc_msg.right_ticks


if __name__ == "__main__":
    try:
        node = Rover6Chassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting rover6_chassis node")
