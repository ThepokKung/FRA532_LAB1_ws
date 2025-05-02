#!/usr/bin/env python3
"""
ground_truth_tf_publisher.py

Subscribe to your ground-truth Odometry (e.g. from Gazebo) and
broadcast a TF from "odom" → "base_link" without any extra conversions.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')

        # If your ground truth odom really lives on /ground_truth/pose, leave it.
        # Otherwise, change this to /ground_truth/odom or /odom_ground_truth
        self.create_subscription(
            Odometry,
            '/ground_truth/pose',
            self.odom_callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("📡 GroundTruth TF Publisher started, listening on /ground_truth/pose")

    def odom_callback(self, msg: Odometry):
        # Build the TF from odom → base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'            # upstream “world” frame
        t.child_frame_id = 'base_link'        # the robot’s footprint

        # Directly copy the pose from the Odometry message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation    = msg.pose.pose.orientation
        # Broadcast!
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
