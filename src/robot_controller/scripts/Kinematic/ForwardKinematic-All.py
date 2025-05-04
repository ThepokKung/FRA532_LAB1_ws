#!/usr/bin/env python3
"""
JointStateForwardKinematicsAll - Forward Kinematics from JointState and IMU using 3 models:
1. Yaw-Rate Model (IMU-based)
2. Single-Track (Bicycle)
3. Double-Track (Differential approximation)
"""
import os
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import tf_transformations


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class JointStateForwardKinematicsAll(Node):
    def __init__(self):
        super().__init__('joint_state_forward_kinematics_all')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('track_width', 0.13)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value

        # Kinematic state
        self.x_yaw = self.y_yaw = self.theta_yaw = 0.0
        self.x_single = self.y_single = self.theta_single = 0.0
        self.x_double = self.y_double = self.theta_double = 0.0

        # Inputs
        self.v_rl = self.v_rr = 0.0
        self.delta_left = self.delta_right = 0.0
        self.yaw_rate = 0.0

        # Fixed timestep
        self.dt = 0.01

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.jointstate_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publishers
        self.odom_pub_yaw = self.create_publisher(Odometry, '/odom_yaw_rate', 10)
        self.odom_pub_single = self.create_publisher(Odometry, '/odom_single_track', 10)
        self.odom_pub_double = self.create_publisher(Odometry, '/odom_double_track', 10)

        # Timer for updates
        self.create_timer(self.dt, self.update_kinematics)

        self.get_logger().info('JointStateForwardKinematicsAll initialized.')

    def jointstate_callback(self, msg: JointState):
        idx_rl = idx_rr = idx_dl = idx_dr = None
        for i, name in enumerate(msg.name):
            if name == 'back_left_wheel': idx_rl = i
            elif name == 'back_right_wheel': idx_rr = i
            elif name == 'front_left_steering_joint': idx_dl = i
            elif name == 'front_right_steering_joint': idx_dr = i
        if None in (idx_rl, idx_rr, idx_dl, idx_dr):
            self.get_logger().warn('Missing joints in JointState')
            return
        self.v_rl = msg.velocity[idx_rl] * self.wheel_radius
        self.v_rr = msg.velocity[idx_rr] * self.wheel_radius
        self.delta_left = msg.position[idx_dl]
        self.delta_right = msg.position[idx_dr]

    def imu_callback(self, msg: Imu):
        self.yaw_rate = msg.angular_velocity.z

    def update_kinematics(self):
        # 1. Yaw-Rate Model
        V = (self.v_rl + self.v_rr) / 2.0
        omega = self.yaw_rate
        mid_theta = self.theta_yaw + 0.5 * omega * self.dt
        self.x_yaw += V * math.cos(mid_theta) * self.dt
        self.y_yaw += V * math.sin(mid_theta) * self.dt
        self.theta_yaw = normalize_angle(self.theta_yaw + omega * self.dt)
        odom_yaw = self.create_odom_msg(self.x_yaw, self.y_yaw, self.theta_yaw, V, omega)
        self.odom_pub_yaw.publish(odom_yaw)

        # 2. Single-Track Model
        Vb = V
        delta = 0.5 * (self.delta_left + self.delta_right)
        omega_b = (Vb / self.wheelbase) * math.tan(delta)
        mid_theta_b = self.theta_single + 0.5 * omega_b * self.dt
        self.x_single += Vb * math.cos(mid_theta_b) * self.dt
        self.y_single += Vb * math.sin(mid_theta_b) * self.dt
        self.theta_single = normalize_angle(self.theta_single + omega_b * self.dt)
        odom_single = self.create_odom_msg(self.x_single, self.y_single, self.theta_single, Vb, omega_b)
        self.odom_pub_single.publish(odom_single)

        # 3. Double-Track Model (Differential approximation)
        Vd = V
        omega_d = (self.v_rr - self.v_rl) / self.track_width
        # store for potential logging or use
        self.v_curr_2Track = Vd
        self.w_curr_2Track = omega_d
        # Midpoint integration
        mid_theta_d = self.theta_double + 0.5 * omega_d * self.dt
        self.x_double += Vd * math.cos(mid_theta_d) * self.dt
        self.y_double += Vd * math.sin(mid_theta_d) * self.dt
        self.theta_double = normalize_angle(self.theta_double + omega_d * self.dt)
        odom_double = self.create_odom_msg(self.x_double, self.y_double, self.theta_double, Vd, omega_d)
        self.odom_pub_double.publish(odom_double)

    def create_odom_msg(self, x, y, theta, lin_vel, ang_vel):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = lin_vel
        odom.twist.twist.angular.z = ang_vel
        return odom


def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardKinematicsAll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
