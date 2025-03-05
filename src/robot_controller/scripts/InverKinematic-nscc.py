#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import isclose, atan, tan


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_nscc_model')

        # พารามิเตอร์หุ่นยนต์
        self.declare_parameter('wheelbase', 0.20)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.declare_parameter('wheel_radius', 0.045)    # r (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value

        # Maximum allowed steering angle (rad)
        self.max_steer = 0.523598767  # 30 degrees

        # Subscriber รับ /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics NSCC Model node has started.")

    def cmd_vel_callback(self, msg):
        V_x = msg.linear.x
        omega = msg.angular.z

        if isclose(omega, 0.0):
            omega = 1e-5

        delta = atan(omega * self.L / V_x) if V_x != 0 else 0.0
        delta = max(min(delta, self.max_steer), -self.max_steer)

        V_wr = V_x / self.r
        tan_delta = tan(delta)
        if delta > self.max_steer:
            delta = self.max_steer
        elif delta < -self.max_steer:
            delta = -self.max_steer

        V_wr = V_x / self.r

        tan_delta = tan(delta)
        delta_left = atan(self.L * tan_delta /
                          (self.L + 0.5 * self.W * tan_delta))
        delta_right = atan(self.L * tan_delta /
                           (self.L - 0.5 * self.W * tan_delta))

        if delta != 0:
            V_rl = V_rr = V_wr
            R = self.L / tan_delta
            V_fl = V_wr / np.abs(V_wr) * np.abs(omega *np.linalg.norm([self.L, R])/self.r)
            V_fr = V_wr / np.abs(V_wr) * np.abs(omega *np.linalg.norm([self.L, R - self.W])/self.r)
        else:
            V_fl = V_fr = V_rl = V_rr = V_wr

        # ส่งค่าควบคุมความเร็วล้อ
        vel_msg = Float64MultiArray()
        vel_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.vel_pub.publish(vel_msg)

        # ส่งค่าควบคุมมุมเลี้ยวล้อหน้า
        steer_msg = Float64MultiArray()
        steer_msg.data = [float(delta_left), float(delta_right)]
        self.steering_pub.publish(steer_msg)

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
