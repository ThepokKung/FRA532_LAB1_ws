#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from math import atan, tan, cos


class InverseKinematicsNSCC(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_nscc')

        # Load robot parameters
        self.declare_parameter('wheel_radius', 0.045)  # 4.5 cm
        self.declare_parameter('wheel_height', 0.01)  # 1 cm
        self.declare_parameter('robot_width', 0.065 * 2)  # 6.5 * 2 cm
        self.declare_parameter('robot_length', 0.10 * 2)  # 10 * 2 cm
        self.declare_parameter('robot_height', 0.10)  # 10 cm
        self.declare_parameter('robot_weight', 3.0)  # 3 kg
        self.declare_parameter('wheelbase', 0.065)  # L = 6.5 cm

        self.L = self.get_parameter('wheelbase').value

        # Subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for wheel speeds
        self.wheel_speed_pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        self.get_logger().info("Inverse Kinematics NSCC Model node has started.")

    def cmd_vel_callback(self, msg):
        X = msg.linear.x  # Forward velocity
        omega = msg.angular.z  # Yaw rate
        W = self.get_parameter('robot_width').value  # Width of the robot

        # Compute turning radius
        if omega != 0:
            if X != 0:
                R = X / omega  # Turning radius
            else:
                R = float('inf')
            delta_left = atan(self.L / (R - W / 2))
            delta_right = atan(self.L / (R + W / 2))
        else:
            R = float('inf')
            delta_left = 0.0
            delta_right = 0.0

        # Compute front and rear wheel speeds
        V_fw = X
        V_rw = X

        # Compute left and right wheel radii
        R_left = R - (W / 2)
        R_right = R + (W / 2)

        # Compute individual wheel speeds under No-Slip Condition Constraints
        if R == float('inf'):  # If robot is moving straight
            V_fl = V_fr = V_fw
            V_rl = V_rr = V_rw
        else:  # If robot is turning, adjust speeds to satisfy no-slip conditions
            V_fl = V_fw * (R_left / R)
            V_fr = V_fw * (R_right / R)
            V_rl = V_rw * (R_left / R)
            V_rr = V_rw * (R_right / R)

        # Publish wheel speeds
        wheel_speeds_msg = Float32MultiArray()
        wheel_speeds_msg.data = [V_fl, V_fr, V_rl, V_rr, delta_left, delta_right]
        self.wheel_speed_pub.publish(wheel_speeds_msg)

        self.get_logger().info(f"Steering Angles: Left={delta_left:.2f} rad, Right={delta_right:.2f} rad")
        self.get_logger().info(f"Wheel Speeds: FL={V_fl:.2f}, FR={V_fr:.2f}, RL={V_rl:.2f}, RR={V_rr:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNSCC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
