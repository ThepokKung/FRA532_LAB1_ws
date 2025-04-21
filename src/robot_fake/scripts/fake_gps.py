#!/usr/bin/env python3

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
        self.noise_level = 1.0  # Adjust the noise level as needed

    def ground_truth_callback(self, msg):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.pose.pose.position.x = msg.pose.pose.position.x + random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.position.y = msg.pose.pose.position.y + random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z #+ random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x #+ random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y #+ random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z #+ random.uniform(-self.noise_level, self.noise_level)
        noisy_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w #+ random.uniform(-self.noise_level, self.noise_level)

        # Normalize the quaternion
        norm = (noisy_msg.pose.pose.orientation.x**2 + noisy_msg.pose.pose.orientation.y**2 +
                noisy_msg.pose.pose.orientation.z**2 + noisy_msg.pose.pose.orientation.w**2) ** 0.5
        noisy_msg.pose.pose.orientation.x /= norm
        noisy_msg.pose.pose.orientation.y /= norm
        noisy_msg.pose.pose.orientation.z /= norm
        noisy_msg.pose.pose.orientation.w /= norm
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
