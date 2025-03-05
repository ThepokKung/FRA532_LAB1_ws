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

class PathTrackingStanleyController(Node):
    def __init__(self):
        super().__init__('path_tracking_stanley_controller')

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

        # Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables
        self.wheelbase = 0.2
        self.track = 0.13
        self.wheel_radius = 0.045

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.path_index = 0

        self.kp_v = 1.0
        self.k_cross = 1.0  # Gain for cross-track error
        self.k_soft = 0.1  # Small constant to avoid instability

        self.last_error = 0.0
        self.yaw_error = 0.0

        self.update_target()
        self.get_logger().info("🚀 Stanley Path Tracking Node Initialized")

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
        if self.last_error < 1:
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
                self.update_target()
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('🏁 Path tracking completed lap')
                return

        # Calculate the error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Stanley Controller
        control_linear = 1.0

        # Compute cross-track error
        front_axle_error = np.dot(
            np.array([np.cos(self.current_yaw + np.pi / 2), np.sin(self.current_yaw + np.pi / 2)]),
            np.array([error_x, error_y])
        )

        # Stanley Control Law
        cross_track_steering = np.arctan2(self.k_cross * front_axle_error, control_linear + self.k_soft)
        control_angular = error_yaw + cross_track_steering

        # Limit the speed
        control_linear = np.clip(control_linear, -1.0, 1.0)
        control_angular = np.clip(control_angular, -1, 1)

        # Publish Control Commands
        self.publish_cmd(control_linear, control_angular)

        self.get_logger().info(f'🎯 Target waypoint: {self.target_x}, {self.target_y}, {self.target_yaw}')
        self.get_logger().info(f'   Error: {distance_error}, Yaw Error: {error_yaw}, Linear: {control_linear}, Angular: {control_angular}')

        # Update the error
        self.last_error = distance_error
        self.yaw_error = error_yaw

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
