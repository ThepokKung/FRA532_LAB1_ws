#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from math import atan, tan


class InverseKinematicsNSCC(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_nscc')

        # Load robot parameters
        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('robot_width', 0.13)
        self.declare_parameter('wheelbase', 0.065)

        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('robot_width').value

        # Subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for wheel speeds
        self.velocities_control_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_control_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics NSCC Model node has started.")

    def cmd_vel_callback(self, msg):
        X = msg.linear.x
        # omega = msg.angular.z
        delta_cmd = msg.angular.z

        # คำนวณรัศมีการเลี้ยว
        if abs(omega) < 1e-6:
            # วิ่งตรง
            delta_left = 0.0
            delta_right = 0.0
            V_fl = V_fr = V_rl = V_rr = X
        else:
            R = X / omega # m
            delta_left = atan(self.L / (R - self.W / 2))
            delta_right = atan(self.L / (R + self.W / 2))
            delta_left = max(min(delta_left, 0.7854), -0.7854)
            delta_right = max(min(delta_right, 0.7854), -0.7854)

            V_fl = X * (R - self.W / 2) / R
            V_fr = X * (R + self.W / 2) / R
            V_rl = V_fl
            V_rr = V_fr

        # ส่งค่าควบคุม
        velocity_control_msg = Float64MultiArray()
        velocity_control_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.velocities_control_pub.publish(velocity_control_msg)

        steering_angles_msg = Float64MultiArray()
        steering_angles_msg.data = [delta_left, delta_right]
        self.steering_control_pub.publish(steering_angles_msg)

        self.get_logger().info(
            f"Steering Angles: Left={delta_left:.2f} rad, Right={delta_right:.2f} rad")
        self.get_logger().info(
            f"Wheel Speeds: FL={V_fl:.2f}, FR={V_fr:.2f}, RL={V_rl:.2f}, RR={V_rr:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNSCC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
