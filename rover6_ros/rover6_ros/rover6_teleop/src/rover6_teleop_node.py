#!/usr/bin/python
from __future__ import division
from __future__ import print_function
import traceback
import math

import tf
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool

from rover6_serial_bridge.msg import Rover6Servos


class Rover6Teleop:
    def __init__(self):
        rospy.init_node(
            "rover6_teleop",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        # class variables
        self.twist_command = Twist()
        self.servo_command = Rover6Servos()
        self.prev_joy_msg = None

        # parameters from launch file
        self.linear_axis = int(rospy.get_param("~linear_axis", 1))
        self.angular_axis = int(rospy.get_param("~angular_axis", 2))
        self.camera_pan_axis = int(rospy.get_param("~camera_pan_axis", 3))
        self.camera_tilt_axis = int(rospy.get_param("~camera_tilt_axis", 4))

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.12)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        # self.wheel_radius = rospy.get_param("~wheel_radius_cm", 32.5)
        # self.wheel_distance = rospy.get_param("~wheel_distance_cm", 22.0)
        # self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 38400.0)
        # self.max_speed_cps = rospy.get_param("~max_speed_cps", 915.0)

        self.pan_right_command = rospy.get_param("~pan_right_command", 90)
        self.pan_left_command = rospy.get_param("~pan_left_command", 0)
        self.pan_center_command = rospy.get_param("~pan_center_command", 43)
        self.tilt_up_command = rospy.get_param("~tilt_up_command", 0)
        self.tilt_down_command = rospy.get_param("~tilt_down_command", 150)
        self.tilt_center_command = rospy.get_param("~tilt_center_command", 105)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_teleop", Twist, queue_size=100)
        self.servo_pub = rospy.Publisher("servos", Rover6Servos, queue_size=100)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)

        # self.tick_to_cm_factor = 2.0 * self.wheel_radius * math.pi / self.ticks_per_rotation
        # self.max_speed_mps = self.max_speed_cps / 1000.0
        # self.max_speed_tps = self.max_speed_mps / self.tick_to_cm_factor  # max speed in ticks per s

        self.max_joy_val = 1.0

        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0
        self.servo_command.camera_pan = -1
        self.servo_command.camera_tilt = -1

        rospy.Timer(rospy.Duration(0.25), self.timer_callback)

    def joy_to_speed(self, scale_factor, value):
        if abs(value) < self.deadzone_joy_val:
            return 0
        joy_val = math.copysign(abs(value) - self.deadzone_joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = scale_factor / max_joy_val_adj * joy_val

        return command

    def joy_to_servo(self, max_command, min_command, center_command, value):
        if abs(value) < self.deadzone_joy_val:
            return None
        joy_val = math.copysign(abs(value) - self.deadzone_joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = (max_command - min_command) / (max_joy_val_adj - (-max_joy_val_adj)) * joy_val + center_command

        return command

    def joy_to_pan_servo(self, value):
        return self.joy_to_servo(self.pan_right_command, self.pan_left_command, self.pan_center_command, value)

    def joy_to_tilt_servo(self, value):
        return self.joy_to_servo(self.tilt_up_command, self.tilt_down_command, self.tilt_center_command, value)

    def did_button_change(self, msg, index):
        return msg.buttons[index] and self.prev_joy_msg.buttons[index] != msg.buttons[index]

    def joystick_msg_callback(self, msg):
        try:
            self.process_joy_msg(msg)
        except BaseException, e:
            traceback.print_exc()
            rospy.signal_shutdown(str(e))

    def set_twist(self, linear_val, angular_val):
        publish_cmd_vel = False
        if self.twist_command.linear.x != linear_val:
            self.twist_command.linear.x = linear_val
            publish_cmd_vel = True
        if self.twist_command.angular.z != angular_val:
            self.twist_command.angular.z = angular_val
            publish_cmd_vel = True
        return publish_cmd_vel

    def set_servos(self, camera_pan_val=None, camera_tilt_val=None):
        if camera_pan_val is None:  # default position
            camera_pan_val = -1
        if camera_tilt_val is None:  # default position
            camera_tilt_val = -1

        publish_servo_vals = False
        if self.servo_command.camera_pan != camera_pan_val:
            self.servo_command.camera_pan = camera_pan_val
            publish_servo_vals = True
        else:
            self.servo_command.camera_pan = -2  # -1 indicates default value, -2 indicates skip

        if self.servo_command.camera_tilt != camera_tilt_val:
            self.servo_command.camera_tilt = camera_tilt_val
            publish_servo_vals = True
        else:
            self.servo_command.camera_tilt = -2  # -1 indicates default value, -2 indicates skip

        return publish_servo_vals

    def process_joy_msg(self, msg):
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return

        # button mapping:
        # 0: A,    1: B,     2: X,      3: Y
        # 4: L1,   5: R1,    6: Select, 7: Start
        # 8: Home, 9: L joy, 10: R joy
        # if self.did_button_change(msg, 0):
        #     pass

        linear_val = self.joy_to_speed(self.linear_scale, msg.axes[self.linear_axis])
        angular_val = self.joy_to_speed(self.angular_scale, msg.axes[self.angular_axis])
        if self.set_twist(linear_val, angular_val):
            self.cmd_vel_pub.publish(self.twist_command)

        camera_pan_val = self.joy_to_pan_servo(msg.axes[self.camera_pan_axis])
        camera_tilt_val = self.joy_to_tilt_servo(msg.axes[self.camera_tilt_axis])

        if self.set_servos(camera_pan_val, camera_tilt_val):
            self.servo_pub.publish(self.servo_command)

        self.prev_joy_msg = msg

    def timer_callback(self, event):
        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(1.0):
            if self.set_twist(0.0, 0.0):
                self.cmd_vel_pub.publish(self.twist_command)
            if self.set_servos():
                self.servo_pub.publish(self.servo_command)

        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(0.5):
            self.cmd_vel_pub.publish(self.twist_command)
            self.servo_pub.publish(self.servo_command)


    def run(self):
        clock_rate = rospy.Rate(1)

        prev_time = rospy.get_rostime()
        while not rospy.is_shutdown():

            self.cmd_vel_pub.publish(self.twist_command)
            self.servo_pub.publish(self.servo_command)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = Rover6Teleop()
        # node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
