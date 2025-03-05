#!/usr/bin/env python3
"""
PID Path Tracking Controller for Ackermann Steering Mobile Robot

This node:
  - Loads a path (list of waypoints) from a YAML file.
  - Subscribes to /ground_truth/pose to obtain the current robot pose.
  - Uses a PID controller to reduce the heading and distance error between the robot and the current target waypoint.
  - Publishes cmd_vel commands.

Assumptions:
  - The YAML file is either a list of waypoints or a dict with key "path".
  - Each waypoint contains at least 'x' and 'y' (optionally 'yaw').

ROS Parameters:
  - Kp, Kp_omega, Ki, Kd: PID gains.
  - lookahead_threshold: Distance threshold to switch to the next waypoint.
  - linear_velocity: Nominal speed.
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


class PathTrackingPID(Node):
    def __init__(self):
        super().__init__('path_tracking_pid')

        # --- Load Path ---
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                data = yaml.safe_load(file)
            # Support both dict with "path" key and direct list.
            self.path = data.get('path', data) if isinstance(data, dict) else data
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # --- PID Gains and Parameters ---
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Kp_omega', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)
        self.Kp = self.get_parameter('Kp').value
        self.Kp_omega = self.get_parameter('Kp_omega').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.declare_parameter('lookahead_threshold', 0.5)
        self.lookahead_threshold = self.get_parameter('lookahead_threshold').value

        self.declare_parameter('linear_velocity', 20.0)
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # --- Internal State Variables ---
        self.integral_error = 0.0
        self.last_distance_error = float('inf')
        self.prev_time = None

        self.path_index = 0
        self.update_target()

        # --- Subscribers and Publishers ---
        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- Timer for control loop ---
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 PID Path Tracking Node Initialized")
        self.get_logger().info(f"🔹 Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, Kp_omega={self.Kp_omega}")
        self.get_logger().info(f"🔹 Lookahead Threshold = {self.lookahead_threshold} m")
        self.get_logger().info(f"🔹 Linear Velocity = {self.linear_velocity} m/s")

    def odom_callback(self, msg: Odometry):
        """Update current pose from /ground_truth/pose."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(q)
        self.current_yaw = yaw

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.dt = 0.1
        else:
            self.dt = current_time - self.prev_time
        self.prev_time = current_time

    def update_target(self):
        """Update the target waypoint based on the current path index."""
        if self.path and self.path_index < len(self.path):
            wp = self.path[self.path_index]
            self.target_x = wp.get('x', 0.0)
            self.target_y = wp.get('y', 0.0)
            self.target_yaw = wp.get('yaw', 0.0)
        else:
            self.target_x = self.target_y = self.target_yaw = 0.0

    def publish_cmd(self, linear: float, angular: float):
        """Publish a Twist message with given linear and angular velocities."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def control_loop(self):
        """Run the PID control loop."""
        # Calculate errors
        error_x = self.target_x - getattr(self, 'current_x', 0.0)
        error_y = self.target_y - getattr(self, 'current_y', 0.0)
        desired_heading = math.atan2(error_y, error_x)
        heading_error = normalize_angle(desired_heading - getattr(self, 'current_yaw', 0.0))
        distance_error = math.hypot(error_x, error_y)

        self.get_logger().debug(f"Target: ({self.target_x:.2f}, {self.target_y:.2f}), "
                                  f"Current: ({getattr(self, 'current_x', 0.0):.2f}, {getattr(self, 'current_y', 0.0):.2f})")
        self.get_logger().debug(f"Distance error: {distance_error:.2f}, Heading error: {math.degrees(heading_error):.2f}°")

        # If close enough, update to next waypoint
        if distance_error < self.lookahead_threshold:
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
                self.update_target()
                self.get_logger().info(f"✅ Reached waypoint {self.path_index - 1}, switching to waypoint {self.path_index}")
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info("🏁 Path tracking completed")
                return

        # Update PID errors
        self.integral_error += distance_error * self.dt
        derivative = (distance_error - self.last_distance_error) / self.dt if self.dt > 0 else 0.0

        # Compute control signals
        linear_cmd = self.Kp * distance_error + self.Ki * self.integral_error + self.Kd * derivative
        angular_cmd = self.Kp_omega * heading_error

        # Limit control outputs
        linear_cmd = np.clip(linear_cmd, -0.5, 0.5)
        angular_cmd = np.clip(angular_cmd, -1.0, 1.0)

        # Publish command
        self.publish_cmd(linear_cmd, angular_cmd)

        self.get_logger().info(f"🎯 Target: ({self.target_x:.2f}, {self.target_y:.2f}, {math.degrees(self.target_yaw):.1f}°)")
        self.get_logger().info(f"Error: distance={distance_error:.2f}, heading={math.degrees(heading_error):.2f}°")
        self.get_logger().info(f"Control: linear={linear_cmd:.2f}, angular={math.degrees(angular_cmd):.2f}°/s")

        # Update last error
        self.last_distance_error = distance_error


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
