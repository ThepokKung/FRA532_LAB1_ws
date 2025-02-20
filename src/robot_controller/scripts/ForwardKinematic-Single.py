#!/usr/bin/python3
"""
forward_kinematics_single_track.py
ใช้ Single-Track (Bicycle) Model ในการคำนวณ odometry จาก wheel speeds
โดยใช้ steering angle จากพารามิเตอร์
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math

class ForwardKinematicsSingleTrack(Node):
    def __init__(self):
        super().__init__('forward_kinematics_single_track')
        # Parameters: wheelbase (L) และ steering_angle (δ)
        self.declare_parameter('wheelbase', 0.10)  # m
        self.declare_parameter('steering_angle', 0.0)  # rad, ให้ค่าสั่งเป็นมุมเลี้ยวของล้อหน้า (สมมติว่าล้อหน้าเลี้ยวเท่ากัน)
        self.L = self.get_parameter('wheelbase').value
        self.delta = self.get_parameter('steering_angle').value

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Subscriber: รับ wheel speeds (Float64MultiArray)
        self.create_subscription(Float64MultiArray, '/wheel_speeds', self.wheel_speeds_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_single_track', 10)

    def wheel_speeds_callback(self, msg):
        if len(msg.data) < 4:
            return
        # ใช้ค่าเฉลี่ยของล้อทั้งหมดเป็นความเร็วเชิงเส้น
        V = sum(msg.data) / 4.0
        # คำนวณอัตราการหมุนจากแบบจักรยาน: ω = V/L * tan(δ)
        omega = V / self.L * math.tan(self.delta)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta += omega * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = V
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsSingleTrack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
