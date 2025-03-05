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

class PathTrackingPurePursuit(Node):
    def __init__(self):
        super().__init__('path_tracking_pure_pursuit')

        # Load the path from the path.yaml file
        path_file = os.path.join(get_package_share_directory('robot_controller'), 'config', 'path.yaml')
        
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                self.path = yaml.safe_load(file)
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.path_index = 0
        self.lookahead_distance = 2.0

        self.kp_v = 1.0
        self.kp_omega = 1.0

        self.last_error = 0.0
        self.yaw_error = 0.0

        self.update_target()
        self.get_logger().info(f'Node PathTrackingPurePursuit Start!!!!!')
             
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
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
        if self.last_error < self.lookahead_distance:
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
                self.update_target()
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('Path tracking completed lap')
                return
            
        # Calculate the error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
        
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # K control
        control_linear = self.kp_v * distance_error
        control_angular = self.kp_omega * error_yaw

        # Limit the speed
        control_linear = np.clip(control_linear, -0.5, 0.5)
        control_angular = np.clip(control_angular, -1, 1)

        # Publish Control Commands
        self.publish_cmd(control_linear, control_angular)

        self.get_logger().info(f'Target: {self.target_x}, {self.target_y}, {self.target_yaw}')
        self.get_logger().info(f'Error: {distance_error}, Yaw Error: {error_yaw}, Linear: {control_linear}, Angular: {control_angular}')
        
        # Update the error
        self.last_error = distance_error
        self.yaw_error = error_yaw

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
