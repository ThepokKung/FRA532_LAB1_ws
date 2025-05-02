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
            self.path = data.get('path', data) if isinstance(
                data, dict) else data
            self.get_logger().info(
                f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # --- Controller Parameters ---
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('K_dd', 1.0)
        self.declare_parameter('min_ld', 0.5)
        self.declare_parameter('max_ld', 2.0)
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('wheelbase', 0.3)

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance').value
        self.K_dd = self.get_parameter('K_dd').value
        self.min_ld = self.get_parameter('min_ld').value
        self.max_ld = self.get_parameter('max_ld').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value

        # --- Internal State ---
        self.current_target_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        if self.path:
            self.start_x = self.path[0]['x']
            self.start_y = self.path[0]['y']

        self.start_x = self.path[0]['x']
        self.start_y = self.path[0]['y']
        self.final_idx = len(self.path) - 1

        self.prev_idx = 0
        self.left_start = False
        self.lap_done = False

        # --- ROS Interfaces ---
        self.create_subscription(
            Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 Pure Pursuit Tracking Node Initialized")

    def search_nearest_point_index(self) -> int:
        dists = [math.hypot(p['x'] - self.robot_x, p['y'] - self.robot_y)
                 for p in self.path]
        return int(np.argmin(dists))

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
        if not self.path or self.lap_done:
            return
        # 0) have we left the start circle yet?
        d0 = math.hypot(self.robot_x - self.start_x, self.robot_y - self.start_y)
        if not self.left_start and d0 > self.min_ld:
            self.left_start = True

        # 1) find the index of the closest path point right now
        idx = self.search_nearest_point_index()

        # 2) wrap detection: only if
        #    a) we've left start,
        #    b) we were already past 90% of the path,
        #    c) and idx just dropped strictly below prev_idx
        min_for_wrap = int(0.9 * self.final_idx)
        if (self.left_start
            and self.prev_idx >= min_for_wrap
            and idx < self.prev_idx):
            self.get_logger().info("🏁 Full lap completed. Stopping.")
            self.pub_cmd(0.0, 0.0)
            self.control_timer.cancel()
            self.lap_done = True
            return

        # 3) compute your dynamic look-ahead length (don’t overwrite the parameter)
        ld = np.clip(self.K_dd * self.linear_velocity, self.min_ld, self.max_ld)

        # 4) scan forward from idx to find the first point ≥ ld away
        lookahead = None
        for p in self.path[idx:]:
            if math.hypot(p['x'] - self.robot_x,
                          p['y'] - self.robot_y) >= ld:
                lookahead = p
                break

        #  ────────────────────────────────────────────
        #  If we didn’t find one:
        #   • Before leaving start → head to the *next* point in the list
        #   • After leaving start  → chase the *final* waypoint (so you close the loop)
        #  ────────────────────────────────────────────
        if lookahead is None:
            if not self.left_start:
                # pick the very next point so you don’t jump to the end
                next_idx = min(idx + 1, self.final_idx)
                lookahead = self.path[next_idx]
            else:
                lookahead = self.path[self.final_idx]

        # 5) transform into robot frame
        dx = lookahead['x'] - self.robot_x
        dy = lookahead['y'] - self.robot_y
        x_r = math.cos(self.robot_yaw)*dx + math.sin(self.robot_yaw)*dy
        y_r = -math.sin(self.robot_yaw)*dx + math.cos(self.robot_yaw)*dy

        # 6) pure-pursuit steering law
        alpha = math.atan2(y_r, x_r)
        delta = math.atan2(2 * self.wheelbase * math.sin(alpha), ld)
        delta = max(-0.6, min(0.6, delta))
        omega = (self.linear_velocity * math.tan(delta)) / self.wheelbase

        # 7) publish
        self.pub_cmd(self.linear_velocity, omega)

        # 8) store for next cycle
        self.prev_idx = idx
        self.get_logger().info(
            f"pp: prev_idx={self.prev_idx}  idx={idx}  left_start={self.left_start}")


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
