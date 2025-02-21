#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from math import atan, tan


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_nscc_model')

        # พารามิเตอร์หุ่นยนต์
        self.declare_parameter('wheelbase', 0.10)    # L (m)
        self.declare_parameter('track_width', 0.13)    # W (m)
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

        # Maximum allowed steering angle (rad)
        self.max_steer = 0.523598767 # 30 degrees

        # Subscriber รับ /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

        self.get_logger().info("Inverse Kinematics NSCC Model node has started.")

    def cmd_vel_callback(self, msg):
        X = msg.linear.x     # ความเร็วเชิงเส้น (m/s)
        # Interpret msg.angular.z as a steering command (δ) directly
        delta_cmd = msg.angular.z
        # Clamp the commanded steering angle to a maximum value
        delta_cmd = max(min(delta_cmd, self.max_steer), -self.max_steer)

        # คำนวณรัศมีการเลี้ยว R โดยใช้สมการ: R = L / tan(δ)
        if abs(delta_cmd) < 1e-6:
            R = float('inf')
            delta_left = 0.0
            delta_right = 0.0
        else:
            R = self.L / tan(delta_cmd)
            # คำนวณมุมเลี้ยวล้อหน้า (Ackermann geometry)
            delta_left = atan(self.L / (R - self.W / 2))
            delta_right = atan(self.L / (R + self.W / 2))

        # จำกัดมุมเลี้ยวไม่ให้เกิน ±max_steer
        delta_left = max(min(delta_left, self.max_steer), -self.max_steer)
        delta_right = max(min(delta_right, self.max_steer), -self.max_steer)

        # คำนวณความเร็วล้อ (ถ้าวิ่งตรง ทุกล้อเท่ากัน)
        if R == float('inf'):
            V_fl = V_fr = V_rl = V_rr = X
        else:
            V_fl = X * (R - self.W / 2) / R
            V_fr = X * (R + self.W / 2) / R
            # ในรถ Ackermann ล้อหลังมักใช้ความเร็วเฉลี่ย X
            V_rl = V_rr = X

        # ส่งค่าควบคุมความเร็วล้อ
        vel_msg = Float64MultiArray()
        vel_msg.data = [V_fl, V_fr, V_rl, V_rr]
        self.vel_pub.publish(vel_msg)

        # ส่งค่าควบคุมมุมเลี้ยวล้อหน้า
        steer_msg = Float64MultiArray()
        steer_msg.data = [delta_left, delta_right]
        self.steering_pub.publish(steer_msg)

        self.get_logger().info(
            f"Commanded δ: {delta_cmd:.3f} rad, R: {R if R != float('inf') else 'inf'}")
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
