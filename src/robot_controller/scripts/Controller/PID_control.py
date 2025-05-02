#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion


class PathTrackingPID(Node):
    def __init__(self):
        super().__init__('path_tracking_pid')

        # Load the path from the path.yaml file
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

        # PID gains (tunable)
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Kp_omega', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('max_speed', 1.0)
        self.Kp = self.get_parameter('Kp').value
        self.Kp_omega = self.get_parameter('Kp_omega').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_speed = self.get_parameter('max_speed').value

        # Lookahead threshold: distance threshold to consider a waypoint reached
        self.declare_parameter('lookahead_threshold', 0.5)
        self.lookahead_threshold = self.get_parameter(
            'lookahead_threshold').value

        # Constant linear velocity (m/s)
        self.declare_parameter('linear_velocity', 1.0)
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # PID state variables
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Current target index in the path
        self.current_target_index = 0

        # Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.path_index = 0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.yaw_error = 0.0
        self.update_target()

        self.get_logger().info("🚀 PID Path Tracking Node Initialized")
        self.get_logger().info(f"🔹 Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        self.get_logger().info(
            f"🔹 Lookahead Threshold = {self.lookahead_threshold} m")
        self.get_logger().info(
            f"🔹 Linear Velocity = {self.linear_velocity} m/s")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def update_target(self):
        self.target_x = self.path[self.path_index]['x']
        self.target_y = self.path[self.path_index]['y']

    def publish_cmd(self, linear, angular):
        """Publishes the velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(twist_msg)

    def control_loop(self):
        if self.path_index >= len(self.path):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('🏁 Path tracking completed lap')
            return

        # Calculate the error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Check if close enough to waypoint
        if distance_error < self.lookahead_threshold:
            self.path_index += 1
            if self.path_index < len(self.path):
                self.update_target()
            return

        # --- PID Control Linear ---
        self.error_sum += distance_error
        error_diff = distance_error - self.last_error

        control_linear = self.Kp * distance_error + \
                        self.Ki * self.error_sum + \
                        self.Kd * error_diff

        # --- PID Control Angular ---
        control_angular = self.Kp_omega * error_yaw

        # --- Limit Velocity ---
        control_linear = np.clip(control_linear, -self.max_speed, self.max_speed)
        control_angular = np.clip(control_angular, -self.max_speed, self.max_speed)

        # Publish control
        self.publish_cmd(control_linear, control_angular)

        self.get_logger().info(f'🎯 Target waypoint: {self.target_x}, {self.target_y}')
        self.get_logger().info(f'   Distance Error: {distance_error:.3f}, Yaw Error: {error_yaw:.3f}')
        self.get_logger().info(f'   Control Linear: {control_linear:.3f}, Control Angular: {control_angular:.3f}')

        # Update
        self.last_error = distance_error
        self.yaw_error = error_yaw

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
