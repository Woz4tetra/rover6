#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

from rover6_chassis.cfg import Rover6ChassisConfig
from rover6_serial_bridge.msg import Rover6Encoder
from rover6_serial_bridge.msg import Rover6Motor
from rover6_serial_bridge.srv import Rover6PidSrv
from rover6_serial_bridge.srv import Rover6SafetySrv

class Rover6Chassis:
    def __init__(self):
        rospy.init_node(
            "rover6_chassis",
            # log_level=rospy.DEBUG
        )

        # rospy.on_shutdown(self.shutdown)

        # robot dimensions
        self.wheel_radius = rospy.get_param("~wheel_radius_cm", 32.5)
        self.wheel_distance = rospy.get_param("~wheel_distance_cm", 1.0)
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 38400.0)
        self.motors_pub_name = rospy.get_param("~motors_pub_name", "motors")

        self.tick_to_cm_factor = 2.0 * self.wheel_radius * math.pi / self.ticks_per_rotation

        # speed parameters
        self.max_speed_cps = rospy.get_param("~max_speed_cps", 915.0)
        self.max_command = rospy.get_param("~max_command", 255)
        self.max_speed_mps = self.max_speed_cps / 1000.0
        self.speed_to_command = self.max_command / self.max_speed_mps

        # TF parameters
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.linear_speed_mps = 0.0
        self.rotational_speed_mps = 0.0

        # Encoder variables
        self.enc_msg = Rover6Encoder()

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # motor message
        self.motors_msg = Rover6Motor()

        # Safety variables
        self.tof_servo_lower_command = rospy.get_param("~tof_servo_lower_command", 180)
        self.tof_servo_upper_command = rospy.get_param("~tof_servo_upper_command", 90)
        self.tof_servo_lower_angle_deg = rospy.get_param("~tof_servo_lower_angle_deg", 275.0)
        self.tof_servo_upper_angle_deg = rospy.get_param("~tof_servo_upper_angle_deg", 360.0)

        self.tof_ground_dist_mm = rospy.get_param("~tof_ground_dist_mm", 28.7)
        self.tof_off_axis_mm = rospy.get_param("~tof_off_axis_mm", 15.0)
        self.tof_front_wall_dist_mm = rospy.get_param("~tof_front_wall_dist_mm", 30.8)
        self.tof_back_wall_dist_mm = rospy.get_param("~tof_back_wall_dist_mm", 15.0)

        # odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.parent_frame
        self.odom_msg.child_frame_id = self.child_frame

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.encoder_sub = rospy.Subscriber("encoders", Rover6Encoder, self.encoder_callback, queue_size=100)

        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.motors_pub = rospy.Publisher("motors_pub_name", Rover6Motor, queue_size=20)

        # Services
        self.pid_service_name = "rover6_pid"
        self.set_pid = None

        self.safety_service_name = "rover6_safety"
        self.set_safety_thresholds = None

        # dynamic reconfigure
        dyn_cfg = Server(Rover6ChassisConfig, lambda config, level: Rover6Chassis.dynamic_callback(self, config, level))

    def dynamic_callback(self, config, level):
        if self.set_pid is None:
            rospy.warning("%s service not ready yet!" % self.pid_service_name)
        else:
            self.call_pid_service(config)

        if self.set_pid is None:
            rospy.warning("%s service not ready yet!" % self.safety_service_name)
        else:
            self.call_safety_service(config)

    def call_pid_service(self, config):
        try:
            self.set_pid(
                config["kp_A"],
                config["ki_A"],
                config["kd_A"],
                config["kp_B"],
                config["ki_B"],
                config["kd_B"]
            )
        except rospy.ServiceException, e:
            rospy.warning("%s service call failed: %s" % (self.pid_service_name, e))

    def call_safety_service(self, config):
        try:
            args = get_stopping_thresholds(
                config["obstacle_threshold_x_mm"],
                config["ledge_threshold_y_mm"],
                config["buffer_x_mm"],
            )
            self.set_safety_thresholds(*args)
        except rospy.ServiceException, e:
            rospy.warning("%s service call failed: %s" % (self.safety_service_name, e))

    def angle_rad_to_tof_servo_command(self, angle_rad):
        angle_rad = angle_rad + (math.pi * 2 if angle_rad <= 0.0 else 0)  # bound to 270...360 deg
        angle_deg = math.degrees(angle_rad)
        y0 = self.tof_servo_lower_command
        y1 = self.tof_servo_upper_command
        x0 = self.tof_servo_lower_angle_deg
        x1 = self.tof_servo_upper_angle_deg

        servo_command = (x1 - x0) / (y1 - y0) * (angle_deg - x0) + y0
        return int(servo_command)


    def calculate_tof_thresholds(obstacle_x, ledge_y, buffer_x, wall_offset):
        gaze_x = obstacle_x + buffer_x + wall_offset  # X coordinate where the sensor is pointing at
        threshold_x = obstacle_x + wall_offset  # X coordinate where threshold starts
        offset_ledge_y = ledge_y + self.tof_ground_dist_mm  # Y threshold relative to the axis of rotation
        sensor_angle = math.atan2(-self.tof_ground_dist_mm, gaze_x)

        # sensor lower threshold accounting for sensor's distance away from the axis of rotation
        obstacle_threshold = threshold_x / math.cos(sensor_angle) - self.tof_off_axis_mm

        # sensor upper threshold accounting for sensor's distance away from the axis of rotation
        ledge_threshold = abs(offset_ledge_y / math.sin(sensor_angle)) - self.tof_off_axis_mm

        servo_command = angle_rad_to_tof_servo_command(sensor_angle)
        assert obstacle_threshold >= 0.0, obstacle_threshold
        assert ledge_threshold >= 0.0, ledge_threshold
        return obstacle_threshold, ledge_threshold, servo_command


    def get_stopping_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm=10.0):
        # obstacle_threshold_x_mm: measured from the maximum point of the front or back of the robot
        # ledge_threshold_y_mm: measured from the ground plane where the robot sits flat on the ground
        # buffer_x_mm: x buffer distance to account for robot stopping time and update rate delay

        front_obstacle_threshold, front_ledge_threshold, front_servo_command = \
            calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                     self.tof_front_wall_dist_mm)

        back_obstacle_threshold, back_ledge_threshold, back_servo_command = \
            calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                     self.tof_back_wall_dist_mm)

        return (
            int(front_obstacle_threshold), int(front_ledge_threshold), int(front_servo_command),
            int(back_obstacle_threshold), int(back_ledge_threshold), int(back_servo_command)
        )

    def twist_callback(self, twist_msg):
        self.linear_speed_mps = twist_msg.linear.x  # m/s
        angular_speed_radps = twist_msg.angular.z  # rad/s

        # arc = angle * radius
        # rotation speed at the wheels
        self.rotational_speed_mps = angular_speed_radps * self.wheel_distance / 2

        left_command = (linear_speed_mps - rotational_speed_mps) * self.speed_to_command
        right_command = (linear_speed_mps + rotational_speed_mps) * self.speed_to_command

        self.motors_msg.left = left_command
        self.motors_msg.right = right_command

        self.motors_pub.publish(self.motors_msg)

    def encoder_callback(self, enc_msg):
        self.enc_msg = enc_msg

    def run(self):
        rospy.wait_for_service(self.pid_service_name)
        self.set_pid = rospy.ServiceProxy(self.pid_service_name, Rover6PidSrv)

        rospy.wait_for_service(self.safety_service_name)
        self.set_safety_thresholds = rospy.ServiceProxy(self.safety_service_name, Rover6SafetySrv)

        clock_rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.compute_odometry()
            self.publish_chassis_data()

            clock_rate.sleep()

    def publish_chassis_data(self):
        odom_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_t)
        self.tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.0),
            odom_quaternion,
            self.enc_msg.header.stamp,
            self.child_frame,
            self.parent_frame
        )

        self.odom_msg.header.stamp = self.enc_msg.header.stamp
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

    def ticks_to_cm(self, ticks):
        return ticks * self.tick_to_cm_factor

    def compute_odometry(self):
        delta_left = self.ticks_to_cm(self.enc_msg.left_ticks - self.prev_left_ticks)
        delta_right = self.ticks_to_cm(self.enc_msg.right_ticks - self.prev_right_ticks)
        delta_dist = (delta_right + delta_left) / 2

        left_speed = self.ticks_to_cm(self.enc_msg.left_speed_ticks_per_s)
        right_speed = self.ticks_to_cm(self.enc_msg.right_speed_ticks_per_s)

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / self.wheel_distance
        self.odom_t += delta_angle

        dx = delta_dist * math.cos(self.odom_t)
        dy = delta_dist * math.sin(self.odom_t)

        self.odom_x += dx
        self.odom_y += dy

        speed = (left_speed + right_speed) / 2
        self.odom_vx = speed * math.cos(self.odom_t)
        self.odom_vy = speed * math.sin(self.odom_t)
        self.odom_vt = (right_speed - left_speed) / (self.wheel_distance / 2)

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
