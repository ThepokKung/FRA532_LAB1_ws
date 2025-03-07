#!/usr/bin/env python3
"""
PathTrackingPurePursuit - Pure Pursuit Path Tracking Controller for Ackermann Steering Mobile Robot

This node:
  - Loads a planned path from a YAML file.
  - Subscribes to the /ground_truth/pose topic.
  - Uses a pure pursuit style controller (using proportional gains) to drive toward the current target waypoint.
  - Publishes the velocity command on the cmd_vel topic.

Assumptions:
  - The path YAML file is either a list of waypoints or a dictionary with key "path".
  - Each waypoint contains at least 'x' and 'y' (and optionally 'yaw').
"""

import os
import math
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class PathTrackingPurePursuit(Node):
    def __init__(self):
        super().__init__('path_tracking_pure_pursuit')

        # --- Load Path ---
        path_file = os.path.join(get_package_share_directory('robot_controller'),
                                 'config', 'path.yaml')
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                data = yaml.safe_load(file)
            # Support both dictionary with 'path' key and direct list.
            self.path = data.get('path', data) if isinstance(data, dict) else data
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # --- Controller Parameters ---
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('Kp_v', 1.0)
        self.declare_parameter('Kp_omega', 0.1)
        self.declare_parameter('switch_threshold', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.Kp_v = self.get_parameter('Kp_v').value  
        self.Kp_omega = self.get_parameter('Kp_omega').value
        self.max_speed = self.get_parameter('max_speed').value

        # --- Internal State ---
        self.path_index = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_distance_error = float('inf')

        # Set initial target waypoint (if path is not empty)
        if self.path:
            self.update_target()
        else:
            self.get_logger().warn("Path is empty; no target waypoint available.")

        # --- ROS Interfaces ---
        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 Pure Pursuit Tracking Node Initialized")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.current_yaw = yaw

    def update_target(self):
        """Update the target waypoint using the current path index."""
        wp = self.path[self.path_index]
        self.target_x = wp.get('x', 0.0)
        self.target_y = wp.get('y', 0.0)
        self.target_yaw = wp.get('yaw', 0.0)

    def publish_cmd(self, linear: float, angular: float):
        """Publish the Twist command message."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)

    def control_loop(self):
        # Compute error between current pose and target waypoint
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance_error = math.hypot(error_x, error_y)
        desired_heading = math.atan2(error_y, error_x)
        heading_error = normalize_angle(desired_heading - self.current_yaw)

        self.get_logger().debug(f"Distance error: {distance_error:.2f}, Heading error: {math.degrees(heading_error):.2f}°")

        # Check if current target is reached (using distance error)
        if distance_error < self.lookahead_distance:
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
                self.update_target()
                self.get_logger().info(f"✅ Reached waypoint {self.path_index - 1}, moving to waypoint {self.path_index}")
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info("🏁 Path tracking completed.")
                return

        # Pure pursuit control (proportional controller)
        linear_cmd = self.Kp_v * distance_error
        angular_cmd = self.Kp_omega * heading_error

        # Limit commands
        linear_cmd = np.clip(linear_cmd, -self.max_speed, self.max_speed)
        angular_cmd = np.clip(angular_cmd, -self.max_speed, self.max_speed)

        self.publish_cmd(linear_cmd, angular_cmd)

        # Log status
        self.get_logger().info(
            f"Target: ({self.target_x:.2f}, {self.target_y:.2f}, yaw: {math.degrees(self.target_yaw):.1f}°) | "
            f"Error: dist={distance_error:.2f}, heading={math.degrees(heading_error):.2f}° | "
            f"Cmd: linear={linear_cmd:.2f}, angular={math.degrees(angular_cmd):.2f}°/s"
        )

        self.last_distance_error = distance_error

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
