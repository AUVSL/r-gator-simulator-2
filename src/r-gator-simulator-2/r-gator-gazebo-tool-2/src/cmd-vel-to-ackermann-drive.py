#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
import math


class CmdVelToAckermannDrive(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')

        self.declare_parameter('twist_cmd_topic', '/cmd_vel')
        self.declare_parameter('ackermann_cmd_topic', '/ackermann_cmd')
        self.declare_parameter('wheelbase', 1.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('message_type', 'ackermann_drive')

        self.twist_cmd_topic = self.get_parameter('twist_cmd_topic').value
        self.ackermann_cmd_topic = self.get_parameter('ackermann_cmd_topic').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.frame_id = self.get_parameter('frame_id').value
        self.message_type = self.get_parameter('message_type').value

        self.subscription = self.create_subscription(
            Twist,
            self.twist_cmd_topic,
            self.cmd_callback,
            1
        )

        if self.message_type == 'ackermann_drive':
            self.publisher = self.create_publisher(AckermannDrive, self.ackermann_cmd_topic, 1)
        else:
            self.publisher = self.create_publisher(AckermannDriveStamped, self.ackermann_cmd_topic, 1)

        self.get_logger().info(
            f"Node 'cmd_vel_to_ackermann_drive' started.\n"
            f"Listening to {self.twist_cmd_topic}, publishing to {self.ackermann_cmd_topic}. "
            f"Frame id: {self.frame_id}, wheelbase: {self.wheelbase}"
        )

    def cmd_callback(self, data):
        v = data.linear.x
        steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)

        if self.message_type == 'ackermann_drive':
            msg = AckermannDrive()
            msg.steering_angle = steering
            msg.speed = v
        else:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.drive.steering_angle = steering
            msg.drive.speed = v

        self.publisher.publish(msg)

    @staticmethod
    def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0

        radius = v / omega
        return math.atan(wheelbase / radius)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToAckermannDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
