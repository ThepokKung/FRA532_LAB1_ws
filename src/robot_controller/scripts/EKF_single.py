#!/usr/bin/python3
"""
ekf_single_track_offset.py

ROS2 Node for Extended Kalman Filter (EKF) Localization using the Single‐Track model.
This node fuses:
  - Odometry from /odom_single_track (control input for prediction)
  - GPS measurements from /fake_gps (measurement update)

State vector: x = [x, y, theta]^T

Motion Model (Predict):
    x_next     = x + v * cos(theta) * dt
    y_next     = y + v * sin(theta) * dt
    theta_next = theta + omega * dt

Observation Model (Update):
    z = [x_gps, y_gps]^T  with  h(x) = [x, y]^T

The node publishes the EKF estimated odometry on /ekf/odom.

Parameters:
  - force_initial_state (bool): If True, EKF state is forced to initial offset.
  - spawn_x_val (float): Offset in x (m) to use as initial state if force_initial_state is True.
  - dt (float): Time step for prediction (s).
  - initial_covariance (float): Diagonal value for initial covariance matrix P.
  - process_noise (list[float]): Diagonal values for Q.
  - measurement_noise (list[float]): Diagonal values for R.
  
TF:
  - The EKF publishes odometry with header.frame_id "odom" and child_frame_id "base_link".
  - Ensure your TF tree (e.g., via static_transform_publisher) maps "world" to "odom" as desired.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class EKFSingleTrackOffset(Node):
    def __init__(self):
        super().__init__('ekf_single_track_offset')
        
        # Declare parameters for EKF tuning and initial state offset
        self.declare_parameter('force_initial_state', True)
        self.declare_parameter('spawn_x_val', 9.073500)  # Offset in x (m)
        self.declare_parameter('dt', 0.1)           # Time step (s)
        self.declare_parameter('initial_covariance', 0.1)
        self.declare_parameter('process_noise', [0.05, 0.05, 0.01])
        self.declare_parameter('measurement_noise', [0.5, 0.5])
        
        self.force_initial_state = self.get_parameter('force_initial_state').value
        self.spawn_x_val = self.get_parameter('spawn_x_val').value
        self.dt = self.get_parameter('dt').value
        
        initial_cov = self.get_parameter('initial_covariance').value
        self.P = np.eye(3) * initial_cov
        
        proc_noise = self.get_parameter('process_noise').value
        self.Q = np.diag(proc_noise)
        
        meas_noise = self.get_parameter('measurement_noise').value
        self.R = np.diag(meas_noise)
        
        # EKF state: [x, y, theta]
        # If force_initial_state is True, set initial state as [spawn_x_val, 0, 0]
        if self.force_initial_state:
            self.x = np.array([self.spawn_x_val, 0.0, 0.0])
        else:
            self.x = np.array([0.0, 0.0, 0.0])
        
        # Last time stamp for prediction step
        self.last_time = None
        
        # Latest control inputs from /odom_single_track (v and omega)
        self.v = 0.0
        self.omega = 0.0
        
        # Flag to initialize EKF from first GPS measurement if not forcing initial state
        self.initialized_gps = False
        
        # Subscribers
        self.create_subscription(Odometry, '/odom_single_track', self.odom_single_track_callback, 10)
        self.create_subscription(Odometry, '/fake_gps', self.fake_gps_callback, 10)
        
        # Publisher for EKF odometry
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        
        self.get_logger().info("EKF Single-Track Offset Node Initialized.")
    
    def odom_single_track_callback(self, msg: Odometry):
        """
        Callback for /odom_single_track.
        Uses twist data (v, omega) as control input and performs the Predict step.
        """
        # Update control inputs from twist
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        
        # Compute dt from header stamp
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            dt = self.dt
        else:
            dt = current_time - self.last_time
            if dt <= 0.0:
                dt = self.dt
        self.last_time = current_time
        
        # Predict step: using motion model
        theta = self.x[2]
        x_pred = np.zeros(3)
        x_pred[0] = self.x[0] + self.v * math.cos(theta) * dt
        x_pred[1] = self.x[1] + self.v * math.sin(theta) * dt
        x_pred[2] = normalize_angle(self.x[2] + self.omega * dt)
        self.x = x_pred
        
        # Compute Jacobian F of the motion model with respect to state
        F = np.array([
            [1.0, 0.0, -self.v * math.sin(theta) * dt],
            [0.0, 1.0,  self.v * math.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
        self.get_logger().debug(f"[Predict] x: {self.x}, P: {self.P}")
    
    def fake_gps_callback(self, msg: Odometry):
        """
        Callback for /fake_gps.
        Uses GPS measurement (x,y) to perform the EKF Update step.
        """
        # Extract GPS measurement z = [x_gps, y_gps]
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.get_logger().debug(f"Received GPS measurement: {z}")
        
        # If force_initial_state is False, initialize state from first GPS measurement
        if not self.force_initial_state and not self.initialized_gps:
            self.x[0] = z[0]
            self.x[1] = z[1]
            self.initialized_gps = True
            self.get_logger().info("Initialized EKF state from first GPS measurement.")
        
        # Observation model: h(x) = [x, y]
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        z_pred = H @ self.x
        y_innov = z - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y_innov
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        
        self.get_logger().debug(f"[Update] x: {self.x}, P: {self.P}")
        
        # Publish the updated EKF odometry
        self.publish_ekf_odometry(msg.header.stamp)
    
    def publish_ekf_odometry(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.pose.position.x = self.x[0]
        odom_msg.pose.pose.position.y = self.x[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (roll=pitch=0)
        qz = math.sin(self.x[2] / 2.0)
        qw = math.cos(self.x[2] / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Optionally fill twist with last control input
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        
        self.ekf_pub.publish(odom_msg)
        self.get_logger().debug(f"Published EKF odom: x={self.x[0]:.2f}, y={self.x[1]:.2f}, theta={math.degrees(self.x[2]):.2f}°")
    
def main(args=None):
    rclpy.init(args=args)
    node = EKFSingleTrackOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down EKF node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
