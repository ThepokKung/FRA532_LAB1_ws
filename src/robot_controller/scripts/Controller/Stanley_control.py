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

        # Stanley controller gains and switching threshold (m)
        self.declare_parameter('Kp_v', 1.0)
        self.declare_parameter('k_cross', 1.0)
        self.declare_parameter('k_soft', 0.1)
        self.declare_parameter('switch_threshold', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steer', 0.7854)
        self.declare_parameter('linear_velocity', 0.5)
        self.Kp_v = self.get_parameter('Kp_v').value
        self.k_cross = self.get_parameter('k_cross').value  # Gain for cross-track error
        self.k_soft = self.get_parameter('k_soft').value    # Softening constant to avoid instability
        self.switch_threshold = self.get_parameter('switch_threshold').value    # Distance error threshold for switching waypoints
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steer').value
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # --- Vehicle and Controller Parameters ---
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        
        # --- Internal State Variables ---
        self.path_index = 0
        self.update_target()  # Set first waypoint as target

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        self.last_distance_error = float('inf')

        self.get_logger().info("🚀 Stanley Path Tracking Node Initialized")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.current_yaw = yaw

        # << ADD THIS >>
        self.current_speed = msg.twist.twist.linear.x

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
        # 1) stop if we’re done
        if self.path_index >= len(self.path)-1:
            self.publish_cmd(0.0, 0.0)
            return

        # 2) front-axle pose
        fx = self.current_x + (self.wheelbase/2.0)*math.cos(self.current_yaw)
        fy = self.current_y + (self.wheelbase/2.0)*math.sin(self.current_yaw)

        # 3) nearest-path-point
        dx = [fx - wp['x'] for wp in self.path]
        dy = [fy - wp['y'] for wp in self.path]
        self.path_index = int(np.argmin(np.hypot(dx, dy)))
        target = self.path[self.path_index]

        # 4) cross-track error signed
        if self.path_index==0:
            x_prev, y_prev = self.path[0]['x'], self.path[0]['y']
        else:
            x_prev, y_prev = self.path[self.path_index-1]['x'], self.path[self.path_index-1]['y']
        x_t, y_t = target['x'], target['y']
        dx_seg, dy_seg = x_t-x_prev, y_t-y_prev
        # projection (optional clamp u to [0,1])
        u = ((fx-x_prev)*dx_seg + (fy-y_prev)*dy_seg) / (dx_seg*dx_seg+dy_seg*dy_seg)
        x_proj, y_proj = x_prev + u*dx_seg, y_prev + u*dy_seg
        e_ct = ((fx-x_proj)*dy_seg - (fy-y_proj)*dx_seg)/math.hypot(dx_seg,dy_seg)

        # 5) heading error
        θ_e = normalize_angle(target['yaw'] - self.current_yaw)

        # 6) Stanley law → steering angle δ
        v = max(self.current_speed, 1e-3)
        δ = θ_e + math.atan2(self.k_cross * e_ct, v + self.k_soft)
        δ = float(np.clip(δ, -self.max_steer, self.max_steer))

        # 7) to angular velocity
        ω = (self.linear_velocity * math.tan(δ)) / self.wheelbase

        # 8) publish
        self.publish_cmd(self.linear_velocity, ω)


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
