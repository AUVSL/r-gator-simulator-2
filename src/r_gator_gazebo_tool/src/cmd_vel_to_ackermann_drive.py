import rclpy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)

def cmd_callback(data):
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    global pub
    global message_type

    if message_type == 'ackermann_drive':
        v = data.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

        msg = AckermannDrive()
        msg.steering_angle = steering
        msg.speed = v

        pub.publish(msg)
    else:
        v = data.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

        msg = AckermannDriveStamped()
        msg.header.stamp = rclpy.time.Time()
        msg.header.frame_id = frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = v

        pub.publish(msg)

def main():
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    global pub
    global message_type

    rclpy.init()

    node = rclpy.create_node('cmd_vel_to_ackermann_drive')

    twist_cmd_topic = node.get_parameter('twist_cmd_topic').get_parameter_value().string_value
    ackermann_cmd_topic = node.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value
    wheelbase = node.get_parameter('wheelbase').get_parameter_value().double_value
    frame_id = node.get_parameter('frame_id').get_parameter_value().string_value
    message_type = node.get_parameter('message_type').get_parameter_value().string_value

    node.create_subscription(Twist, twist_cmd_topic, cmd_callback, 1)

    if message_type == 'ackermann_drive':
        pub = node.create_publisher(AckermannDrive, ackermann_cmd_topic, 1)
    else:
        pub = node.create_publisher(AckermannDriveStamped, ackermann_cmd_topic, 1)

    node.get_logger().info("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f" % (twist_cmd_topic, ackermann_cmd_topic, frame_id, wheelbase))

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
