#!/usr/bin/env python3
"""
InverseKinematics - Basic Model (NSCC) for Ackermann Steering

This ROS2 node subscribes to /cmd_vel (Twist) and computes the required wheel speeds and
front steering angles using an inverse kinematics model for a robot with Ackermann steering.

Parameters (set as ROS2 parameters):
  - wheelbase: Distance between the front and rear axles (L) [m]
  - track_width: Distance between the left and right wheels (W) [m]
  - wheel_radius: Wheel radius (r) [m]
  - max_steer: Maximum allowed steering angle [rad] (hard-coded here to ~30°)

It publishes:
  - Wheel speeds as a Float64MultiArray on /velocity_controllers/commands (order: [FL, FR, RL, RR])
  - Steering angles as a Float64MultiArray on /steering_position_controller/commands ([delta_left, delta_right])
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from math import atan, tan

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_basic_model')

        # Load robot parameters
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.declare_parameter('wheel_radius', 0.045)  # r (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value

        # Maximum allowed steering angle (rad)
        self.max_steer = 0.523598767  # 30 degrees

        # Subscribers and Publishers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.velocities_control_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_control_pub = self.create_publisher(Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics Basic Model node has started.")

    def cmd_vel_callback(self, msg: Twist):
        # Extract inputs
        V_x = msg.linear.x
        omega = msg.angular.z

        # Avoid division by zero for omega and V_x with a small threshold
        if abs(omega) < 1e-6:
            omega = 1e-5
        if abs(V_x) < 1e-6:
            delta = 0.0
        else:
            delta = atan(omega * self.L / V_x)

        # Clamp the steering angle delta
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        # For this basic model, both front wheels steer by the same angle
        delta_left = delta_right = delta

        # Compute the rear wheel speed (rad/s)
        # (Assuming no slip, wheel rotational speed = linear speed / wheel radius)
        wheel_rot_speed = V_x / self.r

        # Compute front wheel speeds
        # If the robot is moving straight, all wheels have the same speed.
        if abs(V_x) < 1e-6:
            V_fl = V_fr = 0.0
        else:
            # Using the norm of [L*omega, V_x] gives a measure of the turning rate
            front_speed = abs(omega * self.L) + abs(V_x)
            # Preserve the sign of the rear wheel speed
            V_fl = V_fr = np.sign(wheel_rot_speed) * (front_speed / self.r)

        # Rear wheels speeds are assumed equal to the computed wheel rotational speed
        V_rl = V_rr = wheel_rot_speed

        # Publish wheel speeds
        vel_msg = Float64MultiArray()
        vel_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.velocities_control_pub.publish(vel_msg)

        # Publish steering angles
        steer_msg = Float64MultiArray()
        steer_msg.data = [delta_left, delta_right]
        self.steering_control_pub.publish(steer_msg)

        self.get_logger().info(f"Steering: Left={delta_left:.3f} rad, Right={delta_right:.3f} rad")
        self.get_logger().info(f"Wheel Speeds: FL={V_fl:.2f}, FR={V_fr:.2f}, RL={V_rl:.2f}, RR={V_rr:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
