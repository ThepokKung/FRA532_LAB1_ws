#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty


class RobotPathTrack(Node):
    def __init__(self):
        super().__init__('robot_path_track')
        self.get_logger().info("Node 'robot_path_track' has been started.")

        # Subscriber to "/odometry/ground_truth"
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth/pose',
            self.odometry_callback,
            10
        )

        # Service server to clear the path
        self.srv = self.create_service(
            srv_type=Empty,  # Use the Empty service type
            srv_name='clear_path',
            callback=self.clear_path_callback
        )

        self.header_gt = None
        self.pose_gt = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"

        # Publisher to "robot_path_track"
        self.publisher = self.create_publisher(Path, 'robot_path_track', 10)

        # Store path data
        self.path = PoseStamped()

    def odometry_callback(self, msg: Odometry):
        post_robot_track = PoseStamped()
        post_robot_track.header = msg.header
        post_robot_track.pose = msg.pose.pose
        self.path_msg.poses.append(post_robot_track)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path_msg)

    def clear_path_callback(self, request, response):
        self.path_msg.poses.clear()
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path_msg)
        self.get_logger().info("Path has been cleared.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotPathTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node is shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
