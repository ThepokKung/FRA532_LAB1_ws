#!/usr/bin/env python3
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
            self.get_logger().info(
                f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # Linerar contoller using
        self.declare_parameter('use_linear_pure', False)
        self.use_linear = self.get_parameter('use_linear').value
        if self.use_linear:
            self.get_logger().info("🔹 Using pure pursuit controller")
        else:
            self.get_logger().info("🔹 Using linear controller")

        # EKF state variables
        self.declare_parameter('ekf_using', False)
        self.ekf_using = self.get_parameter('ekf_using').value

        # Set Contoller parameters
        self.declare_parameter('lookahead_distance', 0.5)  # Default value
        self.declare_parameter('K_dd', 1.0)
        self.declare_parameter('min_ld', 0.5)
        self.declare_parameter('max_ld', 2.0)
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('lookahead_threshold', 0.1)
        
        # Load Controller parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.K_dd = self.get_parameter('K_dd').value
        self.min_ld = self.get_parameter('min_ld').value
        self.max_ld = self.get_parameter('max_ld').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.lookahead_threshold = self.get_parameter('lookahead_threshold').value

        # Set robot parameters
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('track_width', 0.13)    # W (m)

        # Load robot parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value

        # Robot state variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Controller state variables
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Path following parameters
        self.path_index = 0
        
        # Target state variables
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.update_target()  # Initialize target

        # Subscriber
        if self.ekf_using:
            self.get_logger().info("🔹 Using EKF for odometry")
            self.odom_subscriber = self.create_subscription(
                Odometry, '/ekf/odom', self.odom_callback, 10)
        else:
            self.get_logger().info("🔹 Using ground truth for odometry")
            self.odom_subscriber = self.create_subscription(
                Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Log initialization
        self.get_logger().info("🚀 Pure Pursuit Tracking Node Initialized")
        self.get_logger().info(f"🔹 min_ld={self.min_ld}, max_ld={self.max_ld}")
        self.get_logger().info(f"🔹 Linear velocity = {self.linear_velocity} m/s")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def update_target(self):
        if self.path_index < len(self.path):
            self.target_x = self.path[self.path_index]['x']
            self.target_y = self.path[self.path_index]['y'] 
            self.target_yaw = self.path[self.path_index]['yaw']

    def publish_cmd(self, linear, angular):
        """Publishes the velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(twist_msg)

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        # Check if path is completed
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

        if distance_error < self.lookahead_distance:
            self.path_index += 1
            self.update_target()
            return

        # Calculate the lookahead distance
        self.lookahead_distance = self.K_dd * self.linear_velocity
        self.lookahead_distance = np.clip(self.lookahead_distance, self.min_ld, self.max_ld)

        # Steering angle calculation (Beta)
        beta = math.atan2(2 * self.wheelbase * math.sin(error_yaw), self.lookahead_distance)
        # Normalize the steering angle
        beta = self.normalize_angle(beta)
        # Calculate the angular velocity
        control_angular = beta * self.linear_velocity / self.wheelbase

        # if self.use_linear_pure:
        #     # Linear velocity control
        #     control_linear = self.linear_velocity * (1 - (distance_error / self.lookahead_distance)) #Wait for fix
        # else:
        #     # Linear velocity control
        #     control_linear = self.linear_velocity

        control_linear = self.linear_velocity

        # Publish control
        self.publish_cmd(control_linear, control_angular)

        # Log information
        self.get_logger().info(f'🎯 Target waypoint: {self.target_x:.3f}, {self.target_y:.3f}')
        self.get_logger().info(f'   Distance Error: {distance_error:.3f}, Yaw Error: {error_yaw:.3f}')
        self.get_logger().info(f'   Control Linear: {control_linear:.3f}, Control Angular: {control_angular:.3f}')


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
