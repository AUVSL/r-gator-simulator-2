#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64

PI = 3.141592653589793

def get_steer_angle(phi):
    if phi >= 0.0:
        return (PI / 2) - phi
    return (-PI / 2) - phi

class AckermannSteerController(Node):
    def __init__(self):
        super().__init__('ackermann_steer_controller')
        self.declare_parameters_for_wheels()

        # Initialize desired state variables
        self.desired_speed = 0.0  # Initialize desired speed
        self.desired_steer_ang = 0.0  # Initialize desired steering angle

        self.ackermann_cmd_sub = self.create_subscription(
            AckermannDrive, '/ackermann_cmd', self.ackermann_callback, 10)

        self.initialize_wheel_publishers()

        self.last_speed = 0.0
        self.last_steer_ang = 0.0

        self.timer = self.create_timer(0.1, self.control_update)  # 10 Hz control loop

    def declare_parameters_for_wheels(self):
        # Declare parameters for each wheel and steering mechanism
        self.declare_parameter('left_front_wheel.steering_controller', 'left_front_wheel_steering_controller')
        self.declare_parameter('right_front_wheel.steering_controller', 'right_front_wheel_steering_controller')
        self.declare_parameter('left_rear_wheel.controller', 'left_rear_wheel_controller')
        self.declare_parameter('right_rear_wheel.controller', 'right_rear_wheel_controller')
        self.declare_parameter('wheel_diameter', 0.59)  # Default diameter for all wheels

    def initialize_wheel_publishers(self):
        # Initialize publishers for each wheel and steering mechanism
        self.left_front_steer_pub = self.create_publisher(Float64, self.get_parameter('left_front_wheel.steering_controller').get_parameter_value().string_value + '/command', 1)
        self.right_front_steer_pub = self.create_publisher(Float64, self.get_parameter('right_front_wheel.steering_controller').get_parameter_value().string_value + '/command', 1)
        self.left_rear_wheel_pub = self.create_publisher(Float64, self.get_parameter('left_rear_wheel.controller').get_parameter_value().string_value + '/command', 1)
        self.right_rear_wheel_pub = self.create_publisher(Float64, self.get_parameter('right_rear_wheel.controller').get_parameter_value().string_value + '/command', 1)

    def ackermann_callback(self, msg):
        target_msg = AckermannDrive()
        target_msg.speed = msg.speed
        target_msg.steering_angle = msg.steering_angle
        self.desired_speed = msg.speed
        self.desired_steer_ang = msg.steering_angle

        # Update desired speed and steering angle from incoming messages
        self.desired_speed = msg.speed
        self.desired_steer_ang = msg.steering_angle
        rclpy.logging.get_logger('ackermann_steer_controller').info('Received AckermannDrive message: speed = %f, steering_angle = %f' % (msg.speed, msg.steering_angle))

    def control_update(self):
        # Calculate wheel velocities and steering angles based on desired speed and steering angle
        wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        wheel_radius = wheel_diameter / 2.0

        # Assuming a simple proportional controller for demonstration
        speed_error = self.desired_speed - self.last_speed
        steer_ang_error = self.desired_steer_ang - self.last_steer_ang

        # Update speed and steering angle
        self.last_speed += speed_error * 0.1  # P-control for speed
        self.last_steer_ang += steer_ang_error * 0.1  # P-control for steering angle

        # Calculate wheel angular velocities
        wheel_ang_vel = self.last_speed / wheel_radius

        # Calculate steering angles for left and right wheels
        theta_left = get_steer_angle(self.last_steer_ang)
        theta_right = get_steer_angle(self.last_steer_ang)

        # Publish commands to wheel and steering controllers
        self.left_front_steer_pub.publish(Float64(data=theta_left))
        self.right_front_steer_pub.publish(Float64(data=theta_right))
        self.left_rear_wheel_pub.publish(Float64(data=wheel_ang_vel))
        self.right_rear_wheel_pub.publish(Float64(data=wheel_ang_vel))

def main(args=None):
    rclpy.init(args=args)
    controller = AckermannSteerController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()