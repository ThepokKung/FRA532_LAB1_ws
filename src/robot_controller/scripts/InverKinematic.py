#!/usr/bin/python3

import rclpy
from rclpy.node import Node

class InvertKinematic(Node):
    def __init__(self):
        super().__init__('invert_kinematic')
        self.get_logger().info('InvertKinematic node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = InvertKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()