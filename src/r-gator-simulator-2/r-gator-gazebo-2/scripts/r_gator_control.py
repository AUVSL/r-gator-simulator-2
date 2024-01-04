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

        # Standard parameters for each wheel
        self.declare_parameters_for_wheel("left_front_wheel")
        self.declare_parameters_for_wheel("right_front_wheel")
        self.declare_parameters_for_wheel("left_rear_wheel")
        self.declare_parameters_for_wheel("right_rear_wheel")

        # Global parameters
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publishing_frequency", 30.0)
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.sleep_rate = self.create_rate(self.get_parameter("publishing_frequency").value)

        # Initialize transform listener
        self.tf_buffer = tf.Buffer()
        self.listener = tf.TransformListener(self.tf_buffer, self)

        # Retrieve and use the wheel parameters
        self.initialize_wheel_controllers()

        self.ackermann_cmd_lock = threading.Lock()
        self.left_steer_link, self.left_steer_controller, self.left_front_wheel_controller, self.left_front_inv_circ = \
            self.get_wheel_params("left_front_wheel")


        self.right_steer_link, self.right_steer_controller, self.right_front_wheel_controller, self.right_front_inv_circ = \
            self.get_wheel_params("right_front_wheel")

        self.left_rear_link, self.left_rear_wheel_controller, _, self.left_rear_inv_circ = \
            self.get_wheel_params("left_rear_wheel")


        self.right_rear_link, self.right_rear_wheel_controller, _, self.right_rear_inv_circ = \
            self.get_wheel_params("right_rear_wheel")

        # Initialize state variables
        self.init_state_variables()
        self.left_steer_pub = self.create_publisher(Float64, self.left_steer_controller + "/command", 1)
        self.right_steer_pub = self.create_publisher(Float64, self.right_steer_controller + "/command", 1)
        self.left_front_wheel_pub = self.create_publisher(Float64, self.left_front_wheel_controller + "/command", 1)
        self.right_front_wheel_pub = self.create_publisher(Float64, self.right_front_wheel_controller + "/command", 1)
        self.left_rear_wheel_pub = self.create_publisher(Float64, self.left_rear_wheel_controller + "/command", 1)
        self.right_rear_wheel_pub = self.create_publisher(Float64, self.right_rear_wheel_controller + "/command", 1)
        # Subscriptions
        self.gem_ackermann_sub = self.create_subscription(AckermannDrive, "/ackermann_cmd", self.ackermann_callback, 1)

    def declare_parameters_for_wheel(self, wheel):
        prefix = f"r_gator_ackermann_controller.{wheel}."
        self.declare_parameter(prefix + "steering_link_name", "")
        self.declare_parameter(prefix + "steering_controller_name", "")
        self.declare_parameter(prefix + "axle_controller_name", "")
        self.declare_parameter(prefix + "diameter", 0.59)  # Default diameter

    def get_wheel_params(self, wheel):
        prefix = f"r_gator_ackermann_controller.{wheel}."
        steer_link = self.get_parameter(prefix + "steering_link_name").value
        steer_controller = self.get_parameter(prefix + "steering_controller_name").value
        wheel_controller = self.get_parameter(prefix + "axle_controller_name").value
        diameter = float(self.get_parameter(prefix + "diameter").value)
        return steer_link, steer_controller, wheel_controller, 1 / (PI * diameter)

    def initialize_wheel_controllers(self):
        # Retrieve wheel parameters and initialize publishers for each wheel
        wheels = ["left_front_wheel", "right_front_wheel", "left_rear_wheel", "right_rear_wheel"]
        for wheel in wheels:
            steer_link, steer_controller, wheel_controller, inv_circ = self.get_wheel_params(wheel)
            setattr(self, f"{wheel}_steer_pub", self.create_publisher(Float64, steer_controller + "/command", 1))
            setattr(self, f"{wheel}_wheel_pub", self.create_publisher(Float64, wheel_controller + "/command", 1))

    def init_state_variables(self):
        # Initialize all the state variables
        self.steer_ang = 0.0
        self.steer_ang_vel = 0.0
        self.speed = 0.0
        self.accel = 0.0
        self.last_steer_ang = 0.0
        self.last_speed = 0.0
        self.last_cmd_time = 0.0
        self.delta_t = 0.1

    def spin(self):
            last_time = self.get_clock().now()

            while rclpy.ok():
                current_time = self.get_clock().now()
                delta_t = (current_time - last_time).nanoseconds / 1e9
                last_time = current_time

                # if (self.cmd_timeout > 0.0 and (current_time.nanoseconds / 1e9 - self.last_cmd_time > self.cmd_timeout)):
                #     steer_ang_changed, center_y = self.control_steering(self.last_steer_ang, 0.0, 0.001)
                #     self.control_wheels(0.0, 0.0, 0.1, steer_ang_changed, center_y)
                # elif delta_t > 0.0:
                with self.ackermann_cmd_lock:
                    steer_ang = self.steer_ang
                    steer_ang_vel = self.steer_ang_vel
                    speed = self.speed
                    accel = self.accel

                    steer_ang_changed, center_y = self.control_steering(steer_ang, steer_ang_vel, delta_t)
                    self.control_wheels(speed, accel, delta_t, steer_ang_changed, center_y)

                self.sleep_rate.sleep()

    def ackermann_callback(self, ackermann_cmd):
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        with self.ackermann_cmd_lock:
            self.steer_ang = ackermann_cmd.steering_angle
            self.steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self.speed = ackermann_cmd.speed
            self.accel = ackermann_cmd.acceleration
            print("ackermann calledback")

    def control_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Implementing a simple proportional controller for steering
        error = steer_ang - self.last_steer_ang
        if steer_ang_vel_limit > 0:
            # Limit the steering angle velocity
            ang_vel = max(min(error / delta_t, steer_ang_vel_limit), -steer_ang_vel_limit)
        else:
            ang_vel = error / delta_t

        new_steer_ang = self.last_steer_ang + ang_vel * delta_t

        # Update the last steering angle
        self.last_steer_ang = new_steer_ang

        # Calculate the steering angles for left and right wheels
        self.theta_left = get_steer_angle(new_steer_ang)
        self.theta_right = get_steer_angle(new_steer_ang)

        # Publishing the steering commands
        self.left_steer_pub.publish(Float64(data=self.theta_left))
        self.right_steer_pub.publish(Float64(data=self.theta_right))

        # Check if steering angle changed significantly (threshold might need adjustment)
        steer_ang_changed = abs(new_steer_ang - steer_ang) > 0.01

        return steer_ang_changed, new_steer_ang

    def control_wheels(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):
        # Simple proportional controller for acceleration
        error_speed = speed - self.last_speed
        if accel_limit > 0:
            # Limit the acceleration
            accel = max(min(error_speed / delta_t, accel_limit), -accel_limit)
        else:
            accel = error_speed / delta_t

        new_speed = self.last_speed + accel * delta_t

        # Update the last speed
        self.last_speed = new_speed

        # Calculate wheel velocities based on new speed and steering angle
        # Assuming the same speed for all wheels for simplicity
        self.left_front_ang_vel = new_speed * self.left_front_inv_circ
        self.right_front_ang_vel = new_speed * self.right_front_inv_circ
        self.left_rear_ang_vel = new_speed * self.left_rear_inv_circ
        self.right_rear_ang_vel = new_speed * self.right_rear_inv_circ

        # Publishing the wheel speed commands
        self.left_front_wheel_pub.publish(Float64(data=self.left_front_ang_vel))
        self.right_front_wheel_pub.publish(Float64(data=self.right_front_ang_vel))
        self.left_rear_wheel_pub.publish(Float64(data=self.left_rear_ang_vel))
        self.right_rear_wheel_pub.publish(Float64(data=self.right_rear_ang_vel))

def main(args=None):
        rclpy.init(args=args)
        controller = GEMController()
        try:
            controller.spin()
        except KeyboardInterrupt:
            pass
        finally:
            controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()