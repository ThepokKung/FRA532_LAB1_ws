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

        # YAML file path
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )

        # Load the path from the YAML file
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                self.path = yaml.safe_load(file)
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # Using linear PID controller
        self.declare_parameter('use_linear_pid', False)
        self.use_linear_pid = self.get_parameter('use_linear_pid').value

        # EKF state variables
        self.declare_parameter('ekf_using', False)
        self.ekf_using = self.get_parameter('ekf_using').value

        # Robot paremeters
        self.declare_parameter('wheelbase', 0.2)
        self.wheelbase = self.get_parameter('wheelbase').value

        # Set Contoller parameters
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Kp_omega', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('lookahead_threshold', 0.1)  # PID Only
        # Load Controller parameters
        self.Kp = self.get_parameter('Kp').value
        self.Kp_omega = self.get_parameter('Kp_omega').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.lookahead_threshold = self.get_parameter('lookahead_threshold').value

        # Controller state variables
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Robot state variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Path following parameters
        self.path_index = 0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.yaw_error = 0.0

        # Target state variables
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.update_target()  # Initialize target

        # Subscriber
        if self.ekf_using:
            self.get_logger().info("🔹 Using EKF for odometry")
            self.odom_subscriber = self.create_subscription(Odometry, '/ekf/odom', self.odom_callback, 10)
        else:
            self.get_logger().info("🔹 Using ground truth for odometry")
            self.odom_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Log initialization
        self.get_logger().info("🚀 PID Path Tracking Node Initialized")
        self.get_logger().info(f"🔹 Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        self.get_logger().info(f"🔹 Lookahead Threshold = {self.lookahead_threshold} m")
        self.get_logger().info(f"🔹 Linear velocity = {self.linear_velocity} m/s")

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
        self.target_yaw = self.path[self.path_index]['yaw']

    def publish_cmd(self, linear, angular):
        """Publishes the velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(twist_msg)

    def control_loop(self):
        # Check if lab is completed
        if self.path_index >= len(self.path):
            self.get_logger().info('🏁 Path tracking completed lap')
            self.publish_cmd(0.0, 0.0)
            return

        # Calculate the error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        # Normalize angle to [-π, π]
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Check if we need to move to the next waypoint
        if distance_error < self.lookahead_threshold:
            self.path_index += 1
            self.update_target()
            return

        # --- PID Control Linear ---
        self.error_sum += distance_error
        error_diff = distance_error - self.last_error

        if self.use_linear_pid:
            control_linear = (self.Kp * distance_error +
                              self.Ki * self.error_sum +
                              self.Kd * error_diff)
        else:
            control_linear = self.linear_velocity
            # control_linear = min(self.linear_velocity, self.Kp * distance_error)

        # --- PID Control Angular ---
        control_angular = self.Kp_omega * error_yaw

        # --- Limit Velocity ---
        # control_linear = np.clip(control_linear, -self.linear_velocity, self.linear_velocity)
        # control_angular = np.clip(control_angular, -self.linear_velocity, self.linear_velocity)

        # Publish control
        self.publish_cmd(control_linear, control_angular)

        # Log information
        self.get_logger().info(f'🎯 Target waypoint: {self.target_x:.3f}, {self.target_y:.3f}')
        self.get_logger().info(f'   Distance Error: {distance_error:.3f}, Yaw Error: {error_yaw:.3f}')
        self.get_logger().info(f'   Control Linear: {control_linear:.3f}, Control Angular: {control_angular:.3f}')

        # Update previous values for next iteration
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
