import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import random

class RandomAckermannPublisher(Node):

    def __init__(self):
        super().__init__('random_ackermann_publisher')
        self.publisher_ = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        timer_period = 1  # seconds (change as needed)
        self.timer = self.create_timer(timer_period, self.publish_random_ackermann)

    def publish_random_ackermann(self):
        msg = AckermannDrive()
        msg.speed = random.uniform(-1.0, 1.0)  # Random speed value between -1.0 and 1.0
        msg.steering_angle = random.uniform(-0.5, 0.5)  # Random steering angle between -0.5 and 0.5 radians
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    random_ackermann_publisher = RandomAckermannPublisher()
    rclpy.spin(random_ackermann_publisher)
    random_ackermann_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
