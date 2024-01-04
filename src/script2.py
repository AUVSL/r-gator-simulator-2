import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomVelocityPublisher(Node):

    def __init__(self):
        super().__init__('random_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_random_velocity)

    def publish_random_velocity(self):
        msg = Twist()
        msg.linear.x = random.uniform(-100.0, 100.0)
        msg.angular.z = random.uniform(-1.0, 1.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    random_velocity_publisher = RandomVelocityPublisher()
    rclpy.spin(random_velocity_publisher)
    random_velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
