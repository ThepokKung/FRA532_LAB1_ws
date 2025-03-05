#!/usr/bin/env python3
"""
PathTrackingStanleyController - Stanley Controller for Path Tracking

This node loads a path (list of waypoints) from a YAML file, subscribes to the 
/ground_truth/pose topic for the current robot pose, and uses the Stanley method 
to compute control commands (cmd_vel) for path tracking.

The Stanley control law is defined as:
   steering = heading_error + arctan(k_cross * cross_track_error / (v + k_soft))
where:
   - heading_error is the difference between the desired heading (to the target waypoint) and the current heading.
   - cross_track_error is the perpendicular distance from the vehicle's current position to the path.
   - k_cross is the gain for cross-track error and k_soft is a small constant to avoid division by zero.

Assumptions:
  - The YAML file (path.yaml) contains either a list of waypoints or a dictionary with key "path".
  - Each waypoint contains keys 'x', 'y' and optionally 'yaw'.
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


class PathTrackingStanleyController(Node):
    def __init__(self):
        super().__init__('path_tracking_stanley_controller')

        # --- Load Path from YAML ---
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                data = yaml.safe_load(file)
            # Support both dictionary with "path" key and direct list.
            self.path = data.get('path', data) if isinstance(data, dict) else data
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # --- ROS Subscribers and Publishers ---
        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # --- Vehicle and Controller Parameters ---
        self.wheelbase = 0.2       # m
        self.track_width = 0.13    # m

        # Stanley controller gains and switching threshold (m)
        self.kp_v = 1.0
        self.k_cross = 1.0         # Gain for cross-track error
        self.k_soft = 0.1          # Softening constant to avoid instability
        self.switch_threshold = 1.0  # Distance error threshold for switching waypoints

        # --- Internal State Variables ---
        self.path_index = 0
        self.update_target()  # Set first waypoint as target

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.last_distance_error = float('inf')

        self.get_logger().info("🚀 Stanley Path Tracking Node Initialized")

    def odom_callback(self, msg: Odometry):
        """Update the current pose from /ground_truth/pose."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.current_yaw = yaw

    def update_target(self):
        """Update the target waypoint based on the current path index."""
        if self.path_index < len(self.path):
            wp = self.path[self.path_index]
            self.target_x = wp.get('x', 0.0)
            self.target_y = wp.get('y', 0.0)
            self.target_yaw = wp.get('yaw', 0.0)
        else:
            self.target_x = self.target_y = self.target_yaw = 0.0

    def publish_cmd(self, linear: float, angular: float):
        """Publish the velocity command."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        # Calculate position error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance_error = math.hypot(error_x, error_y)
        desired_heading = math.atan2(error_y, error_x)
        heading_error = normalize_angle(desired_heading - self.current_yaw)

        # Switch to next waypoint if close enough
        if distance_error < self.switch_threshold:
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
                self.update_target()
                self.get_logger().info(f"✅ Reached waypoint {self.path_index - 1}; switching to waypoint {self.path_index}")
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info("🏁 Path tracking completed")
                return

        # Stanley Controller
        # Use constant speed (v_const) for computing cross-track steering term.
        v_const = self.kp_v  # Alternatively, set a different constant speed if needed
        # Compute cross-track error by projecting the error vector onto a vector perpendicular to current heading.
        perp_angle = self.current_yaw + math.pi / 2.0
        cross_track_error = np.dot(np.array([math.cos(perp_angle), math.sin(perp_angle)]),
                                    np.array([error_x, error_y]))
        cross_track_steering = math.atan2(self.k_cross * cross_track_error, v_const + self.k_soft)
        control_angular = heading_error + cross_track_steering
        control_linear = v_const  # Use constant speed command

        # Limit commands
        control_linear = np.clip(control_linear, -1.0, 1.0)
        control_angular = np.clip(control_angular, -1.0, 1.0)

        # Publish command
        self.publish_cmd(control_linear, control_angular)

        # Log details
        self.get_logger().info(
            f"Target: ({self.target_x:.2f}, {self.target_y:.2f}, yaw: {math.degrees(self.target_yaw):.1f}°) | "
            f"Error: {distance_error:.2f} m, {math.degrees(heading_error):.2f}° | "
            f"Cmd: linear={control_linear:.2f}, angular={math.degrees(control_angular):.2f}°/s"
        )

        self.last_distance_error = distance_error


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingStanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
