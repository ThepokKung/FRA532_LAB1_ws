#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from math import atan, tan, isclose


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_basic_model')

        # Load robot parameters
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

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
        X = msg.linear.x
        omega = msg.angular.z

        # Calculate turning radius R using the equation: R = X / ω
        if isclose(omega, 0.0, abs_tol=1e-6):
            R = float('inf')
            delta_left = 0.0
            delta_right = 0.0
        else:
            R = X / omega
            # Calculate front wheel steering angles (Ackermann geometry)
            delta_left = atan(self.L / (R - self.W / 2))
            delta_right = atan(self.L / (R + self.W / 2))

        # Calculate wheel speeds (if moving straight, all wheels have the same speed)
        if R == float('inf'):
            V_fl = V_fr = V_rl = V_rr = X
        else:
            V_fl = omega * (R - self.W / 2)
            V_fr = omega * (R + self.W / 2)
            # In Ackermann steering, rear wheels typically use the average speed X
            V_rl = V_rr = X

        # Publish control values
        velocity_control_msg = Float64MultiArray()
        velocity_control_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.velocities_control_pub.publish(velocity_control_msg)

        steering_angles_msg = Float64MultiArray()
        steering_angles_msg.data = [delta_left, delta_right]
        self.steering_control_pub.publish(steering_angles_msg)

        self.get_logger().info(
            f"Commanded ω: {omega:.3f} rad/s, R: {R if R != float('inf') else 'inf'}")
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
