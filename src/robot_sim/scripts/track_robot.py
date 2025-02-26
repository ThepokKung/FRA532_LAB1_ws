#!/usr/bin/python3
"""
ground_truth_tf_publisher_tftrans.py

ROS2 Node for broadcasting TF based on ground_truth odometry.
This node subscribes to an odometry topic (default: /ground_truth/odom)
and broadcasts a transform from "odom" to "base_link" using the pose information
from the ground_truth odometry message.

Parameters:
  - odom_topic (string): Topic name from which ground_truth odometry is received.
    Default is "/ground_truth/odom".

Usage:
  ros2 run <your_package_name> ground_truth_tf_publisher_tftrans.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_transformations import quaternion_from_euler
import math

class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher_tftrans')
        
        # # Declare parameter for the odometry topic (default: /ground_truth/odom)
        # self.declare_parameter('odom_topic', '/ground_truth/odom')
        # self.odom_topic = self.get_parameter('odom_topic').value

        # Subscriber to ground truth odometry
        self.create_subscription(Odometry, "/ground_truth/pose", self.odom_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(f"GroundTruth TF Publisher node started, subscribing to: /ground_truth/pose ")

    def odom_callback(self, msg: Odometry):
        # Create a TransformStamped message from the odometry message
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"       # Fixed frame (adjust if needed)
        t.child_frame_id = "base_link"     # Robot's base frame
        
        # Use position from the odometry message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # If you want to compute quaternion from Euler (for example, if you want to adjust yaw),
        # you can extract yaw and then use tf_transformations.quaternion_from_euler.
        # Here, we assume roll=pitch=0 and use the yaw from the message's quaternion.
        # Alternatively, you may directly use the quaternion from the odometry message.
        # For demonstration, we re-calculate the quaternion from a yaw angle extracted from msg:
        # Note: In many cases msg.pose.pose.orientation is already valid.
        # For example, if we want to set roll=0, pitch=0, and recalc yaw:
        # Extract yaw from the received quaternion:
        q = msg.pose.pose.orientation
        # Compute yaw from quaternion (assuming roll and pitch are small)
        yaw = 2 * math.atan2(q.z, q.w)
        q_new = quaternion_from_euler(0, 0, yaw)
        t.transform.rotation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Broadcasting ground truth TF.")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down GroundTruth TF Publisher node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
