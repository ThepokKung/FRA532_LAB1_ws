#!/usr/bin/python3
"""
ekf_yaw_rate_node.py

ROS2 Node for Extended Kalman Filter (EKF) Localization using the Yaw Rate Model.

State vector: x = [x, y, theta]^T

Motion Model (Predict):
    x_{t+1} = x_t + v*cos(theta_t)*dt
    y_{t+1} = y_t + v*sin(theta_t)*dt
    theta_{t+1} = theta_t + omega*dt

Control input is received from /cmd_vel (Twist message) where:
    - linear.x gives v (m/s)
    - angular.z gives ω (rad/s)

Measurement is received from /odom_yaw_rate (Odometry message) as:
    z = [x_meas, y_meas, theta_meas]

The node publishes the filtered state as Odometry on /ekf/odom.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class EKFYawRate(Node):
    def __init__(self):
        super().__init__('ekf_yaw_rate')
        
        # Initial state: [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        # Initial covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance Q
        self.Q = np.diag([0.1, 0.1, 0.01])
        # Measurement noise covariance R (for odometry measurement)
        self.R = np.diag([0.5, 0.5, 0.1])
        
        # Fixed time step (s)
        self.dt = 0.1
        
        # Latest control input (v, omega) from /cmd_vel
        self.v = 0.0
        self.omega = 0.0
        
        # Subscribers: Control input and measurement
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom_yaw_rate', self.odom_measurement_callback, 10)
        
        # Publisher for EKF odometry estimate
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/yaw_rate', 10)
        
        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("EKF Yaw Rate Node Initialized.")
    
    def cmd_vel_callback(self, msg: Twist):
        # Update control inputs from /cmd_vel
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.get_logger().debug(f"Control Input Updated: v = {self.v:.2f} m/s, ω = {self.omega:.2f} rad/s")
    
    def odom_measurement_callback(self, msg: Odometry):
        # Measurement update: use the odometry from /odom_yaw_rate
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        self.get_logger().debug(f"Received Measurement: {z}")
        
        # Predict step: use current control inputs
        self.predict(self.dt)
        # Update step: fuse measurement z
        self.update(z)
        # Publish EKF estimated odometry
        self.publish_ekf_odometry(msg.header.stamp)
    
    def predict(self, dt):
        # Current orientation
        theta = self.x[2]
        # Motion model: x_{t+1} = x_t + v*cos(theta)*dt, etc.
        x_pred = np.zeros(3)
        x_pred[0] = self.x[0] + self.v * math.cos(theta) * dt
        x_pred[1] = self.x[1] + self.v * math.sin(theta) * dt
        x_pred[2] = self.x[2] + self.omega * dt
        self.x = x_pred
        
        # Jacobian of motion model with respect to state
        F = np.array([
            [1, 0, -self.v * math.sin(theta) * dt],
            [0, 1,  self.v * math.cos(theta) * dt],
            [0, 0, 1]
        ])
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
        
        self.get_logger().debug(f"Predict: x = {self.x}, P = {self.P}")
    
    def update(self, z):
        # Measurement model: h(x) = [x, y, theta]
        H = np.eye(3)  # Linear observation model
        z_pred = H @ self.x
        # Innovation
        y_innov = z - z_pred
        # Normalize yaw innovation
        y_innov[2] = normalize_angle(y_innov[2])
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        # State update
        self.x = self.x + K @ y_innov
        # Covariance update
        self.P = (np.eye(3) - K @ H) @ self.P
        
        self.get_logger().debug(f"Update: x = {self.x}, P = {self.P}")
    
    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle (assuming q represents orientation)
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    
    def publish_ekf_odometry(self, stamp):
        # Publish the EKF estimated state as an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x[0]
        odom_msg.pose.pose.position.y = self.x[1]
        odom_msg.pose.pose.position.z = 0.0
        # Convert yaw to quaternion (only yaw component)
        qz = math.sin(self.x[2] / 2.0)
        qw = math.cos(self.x[2] / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Optionally publish the control inputs in twist (for reference)
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        
        self.ekf_pub.publish(odom_msg)
        self.get_logger().debug(f"Published EKF odom: x={self.x[0]:.2f}, y={self.x[1]:.2f}, theta={math.degrees(self.x[2]):.2f}°")
    
def main(args=None):
    rclpy.init(args=args)
    node = EKFYawRate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down EKF node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
