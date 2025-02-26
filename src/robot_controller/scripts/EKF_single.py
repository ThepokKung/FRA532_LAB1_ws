#!/usr/bin/python3
"""
ekf_single_track.py

ROS2 Node for Extended Kalman Filter (EKF) Localization using the Single‐Track model.
This node fuses the odometry computed by the Single‐Track model (from /odom_single_track)
with GPS measurements from /fake_gps to obtain a filtered state estimate.

State vector: x = [x, y, theta]^T

Motion Model (Predict):
    x_next = x + v*cos(theta)*dt
    y_next = y + v*sin(theta)*dt
    theta_next = theta + omega*dt

Observation Model (Update):
    z = [x_gps, y_gps]^T,  h(x) = [x, y]^T

The node publishes the EKF estimated odometry on /ekf/odom.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class EKFSingleTrack(Node):
    def __init__(self):
        super().__init__('ekf_single_track')
        
        # EKF state: [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        # Initial covariance matrix (3x3)
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance Q (tunable)
        self.Q = np.diag([0.05, 0.05, 0.01])
        # Measurement noise covariance R for GPS measurements [x,y]
        self.R = np.diag([0.5, 0.5])
        
        # Last timestamp for prediction step
        self.last_time = None
        
        # Latest control inputs from /odom_single_track (from Single-track odometry)
        self.v = 0.0     # linear velocity (m/s)
        self.omega = 0.0 # angular velocity (rad/s)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom_single_track', self.odom_single_track_callback, 10)
        self.create_subscription(Odometry, '/fake_gps', self.fake_gps_callback, 10)
        
        # Publisher for EKF estimated odometry
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        
        self.get_logger().info("EKF Single-Track Node Initialized.")
    
    def odom_single_track_callback(self, msg: Odometry):
        """
        Callback for /odom_single_track.
        Use this message as control input (v and omega) and perform the Predict step.
        """
        # Extract pose from odom_single_track (used only for control inputs)
        # (Though this message also contains pose, we use the twist for control input.)
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        
        # Get current time in seconds
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            dt = 0.1
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        
        # Predict step using motion model:
        theta = self.x[2]
        # Predicted state:
        x_pred = np.zeros(3)
        x_pred[0] = self.x[0] + self.v * math.cos(theta) * dt
        x_pred[1] = self.x[1] + self.v * math.sin(theta) * dt
        x_pred[2] = self.x[2] + self.omega * dt
        self.x = x_pred
        
        # Compute Jacobian F of motion model with respect to state:
        F = np.array([
            [1, 0, -self.v * math.sin(theta) * dt],
            [0, 1,  self.v * math.cos(theta) * dt],
            [0, 0, 1]
        ])
        
        # Covariance prediction:
        self.P = F @ self.P @ F.T + self.Q
        
        self.get_logger().debug(f"Predict: x = {self.x}, P = {self.P}")
        # (Optionally, you may publish the predicted state here if desired.)
    
    def fake_gps_callback(self, msg: Odometry):
        """
        Callback for /fake_gps.
        Use the GPS measurement (assumed to be x,y) to perform the EKF Update step.
        """
        # Extract measurement from GPS message
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.get_logger().debug(f"Received GPS measurement: {z}")
        
        # Observation model: h(x) = [x, y]
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        z_pred = H @ self.x
        # Innovation:
        y_innov = z - z_pred
        
        # Innovation covariance:
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain:
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state:
        self.x = self.x + K @ y_innov
        # Update covariance:
        self.P = (np.eye(3) - K @ H) @ self.P
        
        self.get_logger().debug(f"Update: x = {self.x}, P = {self.P}")
        
        # Publish the EKF estimated odometry
        self.publish_ekf_odometry(msg.header.stamp)
    
    def publish_ekf_odometry(self, stamp):
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
        
        # Optionally, publish the twist (here we can use the control inputs)
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        
        self.ekf_pub.publish(odom_msg)
        self.get_logger().debug(f"Published EKF odom: x={self.x[0]:.2f}, y={self.x[1]:.2f}, theta={math.degrees(self.x[2]):.2f}°")
    
def main(args=None):
    rclpy.init(args=args)
    node = EKFSingleTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down EKF Single-Track node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
