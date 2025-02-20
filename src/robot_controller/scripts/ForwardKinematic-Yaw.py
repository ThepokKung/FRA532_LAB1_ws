#!/usr/bin/python3
"""
forward_kinematics_yaw_rate.py
ใช้ Yaw Rate Model ในการคำนวณ odometry จาก wheel speeds
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math

class ForwardKinematicsYawRate(Node):
    def __init__(self):
        super().__init__('forward_kinematics_yaw_rate')
        # Parameter: ระยะห่างระหว่างล้อซ้ายและขวา (track width)
        self.declare_parameter('track_width', 0.13)  # m
        self.W = self.get_parameter('track_width').value

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Subscriber: รับ wheel speeds (Float64MultiArray: [FL, FR, RL, RR])
        self.create_subscription(Float64MultiArray, '/velocity_controllers/commands', self.wheel_speeds_callback, 10)
        # Publisher: odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_yaw_rate', 10)

    def wheel_speeds_callback(self, msg):
        if len(msg.data) < 4:
            return
        # คำนวณความเร็วของล้อซ้ายและขวา
        v_left = (msg.data[0] + msg.data[2]) / 2.0
        v_right = (msg.data[1] + msg.data[3]) / 2.0

        V = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.W

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
    node = ForwardKinematicsYawRate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
