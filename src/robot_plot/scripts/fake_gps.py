import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps')
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth/pose',
            self.ground_truth_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/fake_gps', 10)
        self.noise_level = 0.0001  # Adjust the noise level as needed

    def ground_truth_callback(self, msg):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.pose.pose.position.x = msg.pose.pose.position.x + random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.position.y = msg.pose.pose.position.y + random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z + random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.covariance = msg.pose.covariance
        self.publisher.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()