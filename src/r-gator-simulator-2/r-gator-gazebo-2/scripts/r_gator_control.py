#!/usr/bin/env python3

import math
import numpy as np
import threading
import rclpy
from rclpy.node import Node
import tf2_ros as tf
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64

PI = 3.141592653589739

def get_steer_angle(phi):
    if phi >= 0.0:
        return (PI / 2) - phi
    return (-PI / 2) - phi

class GEMController(Node):
    def __init__(self):
        super().__init__("gem_ackermann_controller")

        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publishing_frequency", 30.0)
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.sleep_rate = self.create_rate(self.get_parameter("publishing_frequency").value)

        self.tf_buffer = tf.Buffer()
        self.listener = tf.TransformListener(self.tf_buffer, self)

        # Assuming that these parameters are loaded from a YAML file
        self.left_steer_link, self.left_steer_controller, self.left_front_wheel_controller, self.left_front_inv_circ = \
            self.get_wheel_params("left_front_wheel")

        self.right_steer_link, self.right_steer_controller, self.right_front_wheel_controller, self.right_front_inv_circ = \
            self.get_wheel_params("right_front_wheel")

        self.left_rear_link, self.left_rear_wheel_controller, self.left_rear_inv_circ = \
            self.get_wheel_params("left_rear_wheel")

        self.right_rear_link, self.right_rear_wheel_controller, self.right_rear_inv_circ = \
            self.get_wheel_params("right_rear_wheel")

        self.right_rear_link = self.right_rear_link

        self.left_steer_pub = self.create_publisher(Float64, self.left_steer_controller + "/command", 1)
        self.right_steer_pub = self.create_publisher(Float64, self.right_steer_controller + "/command", 1)
        self.left_front_wheel_pub = self.create_publisher(Float64, self.left_front_wheel_controller + "/command", 1)
        self.right_front_wheel_pub = self.create_publisher(Float64, self.right_front_wheel_controller + "/command", 1)
        self.left_rear_wheel_pub = self.create_publisher(Float64, self.left_rear_wheel_controller + "/command", 1)
        self.right_rear_wheel_pub = self.create_publisher(Float64, self.right_rear_wheel_controller + "/command", 1)
        self.gem_ackermann_sub = self.create_subscription(AckermannDrive, "ackermann_cmd", self.ackermann_callback, 1)

        self.ackermann_cmd_lock = threading.Lock()

        self.steer_ang = 0.0
        self.steer_ang_vel = 0.0
        self.speed = 0.0
        self.accel = 0.0
        self.last_steer_ang = 0.0
        self.theta_left = 0.0
        self.theta_left_old = 0.0
        self.theta_right = 0.0
        self.theta_right_old = 0.0
        self.last_speed = 0.0
        self.last_accel_limit = 0.0
        self.left_front_ang_vel = 0.0
        self.right_front_ang_vel = 0.0
        self.left_rear_ang_vel = 0.0
        self.right_rear_ang_vel = 0.0

    def get_wheel_params(self, wheel):
        prefix = f"r_gator_ackermann_controller.{wheel}."
        steer_link = self.get_parameter(prefix + "steering_link_name").value
        steer_controller = self.get_parameter(prefix + "steering_controller_name").value
        wheel_controller = self.get_parameter(prefix + "axle_controller_name").value
        diameter = float(self.get_parameter(prefix + "diameter").value)
        return steer_link, steer_controller, wheel_controller, 1 / (PI * diameter)

    def spin(self):
        last_time = self.get_clock().now()

        while not rclpy.is_shutdown():
            current_time = self.get_clock().now()
            delta_t = (current_time - last_time).nanoseconds / 1e9
            last_time = current_time

            if (self.cmd_timeout > 0.0 and (current_time.nanoseconds / 1e9 - self.last_cmd_time > self.cmd_timeout)):
                steer_ang_changed, center_y = self.control_steering(self.last_steer_ang, 0.0, 0.001)
                self
