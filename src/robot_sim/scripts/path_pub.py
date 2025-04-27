#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import os
import yaml
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class PathPubNode(Node):
    def __init__(self):
        super().__init__('path_pub')
        self.get_logger().info('PathPub node has been initialized.')

        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )

        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                self.path = yaml.safe_load(file)
            self.get_logger().info(
                f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        self.path_pub = self.create_publisher(Path, 'path', 10)

        self.publish_path()

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.path:
            pose = PoseStamped()
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = waypoint['yaw']
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"✅ Published path with {len(path_msg.poses)} waypoints.")


def main(args=None):
    rclpy.init(args=args)
    node = PathPubNode()
    
    # If you want the node to run once and exit
    rclpy.shutdown()
    
    # Alternative: If you want the node to keep running
    # rclpy.spin(node)


if __name__ == '__main__':
    main()
