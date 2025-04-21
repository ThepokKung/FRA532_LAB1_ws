#!/usr/bin/env python3
"""
PathTrackingPurePursuit - Pure Pursuit Path Tracking Controller for Ackermann Steering Mobile Robot

This node:
  - Loads a planned path from a YAML file.
  - Subscribes to the /ground_truth/pose topic.
  - Uses a pure pursuit style controller to drive toward the current target waypoint.
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
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('K_dd', 1.0)
        self.declare_parameter('min_ld', 0.5)
        self.declare_parameter('max_ld', 2.0)
        self.declare_parameter('linear_velo_pure', 0.5)
        self.declare_parameter('wheelbase', 0.3)
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.K_dd = self.get_parameter('K_dd').value  
        self.min_ld = self.get_parameter('min_ld').value
        self.max_ld = self.get_parameter('max_ld').value
        self.linear_velo_pure = self.get_parameter('linear_velo_pure').value
        self.wheelbase = self.get_parameter('wheelbase').value

        # --- Internal State ---
        self.current_target_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # --- ROS Interfaces ---
        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 Pure Pursuit Tracking Node Initialized")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.robot_yaw = yaw

    def pub_cmd(self, linear: float, angular: float):
        """Publish the Twist command message."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if self.current_target_idx >= len(self.path):
            # self.current_target_idx = 0  # Reset index to loop the path
            self.pub_cmd(0.0, 0.0)
            self.get_logger().info("🏁 Path tracking completed.")
            return # Stop
        
        # Search nearest point index
        # self.serch_nearest_point_index()

        # Implement Here
        target = self.path[self.current_target_idx]
        target_x, target_y = target['x'], target['y']

        # Distance Calculation
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)

        # If distance < lookahead_distance, it moves to the next waypoint.
        if distance_error < self.lookahead_distance:
            self.current_target_idx += 1
            self.get_logger().info(f"✅ Reached waypoint {self.current_target_idx - 1}, moving to waypoint {self.current_target_idx}")

        # Heading Angle Calculation
        target_yaw = math.atan2(dy, dx)
        alpha = target_yaw - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        alpha = self.normalize_angle(alpha)

        # Steering Angle Calculation (β)
        self.lookahead_distance = np.clip(self.K_dd * self.linear_velo_pure, self.min_ld, self.max_ld)
        beta = math.atan2(2 * self.wheelbase * math.sin(alpha) / self.lookahead_distance, 1.0)
        beta = max(-0.6, min(beta, 0.6))

        # Angular Velocity Calculation (ω)
        angular_velocity = (self.linear_velo_pure * math.tan(beta)) / self.wheelbase

        # Publish cmd_vel
        self.pub_cmd(self.linear_velo_pure, angular_velocity)
        
        # Log status
        self.get_logger().info(
            f"Target: ({target_x:.2f}, {target_y:.2f}) | "
            f"Error: dist={distance_error:.2f}, alpha={math.degrees(alpha):.2f}° | "
            f"Cmd: linear={self.linear_velo_pure:.2f}, angular={math.degrees(angular_velocity):.2f}°/s"
        )

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
