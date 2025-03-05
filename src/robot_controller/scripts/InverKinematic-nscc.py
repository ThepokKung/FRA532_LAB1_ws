#!/usr/bin/env python3
"""
InverseKinematics - NSCC Model for Ackermann Steering

This node subscribes to /cmd_vel (Twist) and computes wheel speeds and front steering angles
based on an inverse kinematics model for an Ackermann-steered robot.

Parameters (declared as ROS2 parameters):
  - wheelbase: Distance between front and rear axles (L) [m]
  - track_width: Distance between the left and right wheels (W) [m]
  - wheel_radius: Radius of the wheels (r) [m]
  - max_steer: Maximum allowed steering angle [rad] (hard-coded here)

It publishes:
  - Wheel speeds on /velocity_controllers/commands as a Float64MultiArray (order: [FL, FR, RL, RR])
  - Steering commands on /steering_position_controller/commands as a Float64MultiArray ([delta_left, delta_right])
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from math import isclose, atan, tan

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_nscc_model')

        # Robot parameters
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.declare_parameter('wheel_radius', 0.045)  # r (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value

        # Maximum allowed steering angle (rad)
        self.max_steer = 0.523598767  # 30 degrees

        # Subscriber for /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers for wheel speeds and steering angles
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics NSCC Model node has started.")

    def cmd_vel_callback(self, msg):
        V_x = msg.linear.x
        omega = msg.angular.z

        # Avoid division by zero by setting a small value if omega is close to zero.
        if isclose(omega, 0.0):
            omega = 1e-5

        # Compute steering angle delta from the inverse kinematics model
        # When V_x is zero, set delta to zero.
        delta = atan(omega * self.L / V_x) if not isclose(V_x, 0.0) else 0.0

        # Clamp delta to the maximum allowed steering angle.
        delta = max(min(delta, self.max_steer), -self.max_steer)

        # Compute wheel rotational speed (rad/s)
        V_wr = V_x / self.r

        # Compute tan(delta) once
        tan_delta = tan(delta)

        # Compute left/right steering angles (Ackermann geometry)
        # delta_left = atan( L * tan(delta) / (L + 0.5 * W * tan(delta)) )
        # delta_right = atan( L * tan(delta) / (L - 0.5 * W * tan(delta)) )
        delta_left = atan(self.L * tan_delta / (self.L + 0.5 * self.W * tan_delta))
        delta_right = atan(self.L * tan_delta / (self.L - 0.5 * self.W * tan_delta))

        # Compute wheel speeds:
        # For nonzero delta, we use the fact that turning radius R = L / tan(delta)
        if not isclose(delta, 0.0):
            # Rear wheel speeds are assumed equal
            V_rl = V_rr = V_wr
            R = self.L / tan_delta  # Turning radius based on the steering angle
            # For front wheels, we compute speeds based on the distance to the turning center.
            # Norm calculations: sqrt(L^2 + R^2) and sqrt(L^2 + (R - W)^2)
            norm_left = math.sqrt(self.L**2 + R**2)
            norm_right = math.sqrt(self.L**2 + (R - self.W)**2)
            # Compute front wheel speeds to match the turning rate
            # The sign of V_wr is used to preserve direction
            V_fl = np.sign(V_wr) * abs(omega * norm_left / self.r)
            V_fr = np.sign(V_wr) * abs(omega * norm_right / self.r)
        else:
            # Straight motion: all wheels move at the same rotational speed
            V_fl = V_fr = V_rl = V_rr = V_wr

        # Publish wheel speeds
        vel_msg = Float64MultiArray()
        vel_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.vel_pub.publish(vel_msg)

        # Publish steering angles (front wheels)
        steer_msg = Float64MultiArray()
        steer_msg.data = [float(delta_left), float(delta_right)]
        self.steering_pub.publish(steer_msg)

        self.get_logger().info(
            f"Steering: Left={delta_left:.3f} rad, Right={delta_right:.3f} rad")
        self.get_logger().info(
            f"Wheel Speeds: FL={V_fl:.2f}, FR={V_fr:.2f}, RL={V_rl:.2f}, RR={V_rr:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
