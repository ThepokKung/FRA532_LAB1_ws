#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import isclose, atan2, atan


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_basic_model')

        # Load robot parameters
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.declare_parameter('wheel_radius', 0.045)    # r (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value

        # Maximum allowed steering angle (rad)
        self.max_steer = 0.523598767    # 30 degrees

        # Subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for wheel speeds
        self.velocities_control_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_control_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics Basic Model node has started.")

    def cmd_vel_callback(self, msg):
        V_x = msg.linear.x
        omega = msg.angular.z

        if omega == 0:
            omega = 1e-5

        if V_x == 0:
            delta = 0.0
        else:
            delta = atan(omega * self.L / V_x)

        if delta > self.max_steer:
            delta = self.max_steer
        elif delta < -self.max_steer:
            delta = -self.max_steer

        delta_left = delta_right = delta

        V_wr = V_rl = V_rr = V_x / self.r
        if V_wr == 0:
            V_fl = V_fr = 0.0
        else:
            V_fl = V_fr = V_wr / np.abs(V_wr) * \
                np.linalg.norm([self.L * omega, V_x])/self.r

        # Publish control values
        velocity_control_msg = Float64MultiArray()
        velocity_control_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.velocities_control_pub.publish(velocity_control_msg)

        steering_angles_msg = Float64MultiArray()
        steering_angles_msg.data = [delta_left, delta_right]
        self.steering_control_pub.publish(steering_angles_msg)

        # self.get_logger().info(f"Commanded ω: {omega:.3f} rad/s, R: {R if R != float('inf') else 'inf'}")
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
