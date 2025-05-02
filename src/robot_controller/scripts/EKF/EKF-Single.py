#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math
from tf_transformations import quaternion_from_euler

class EKFSingleTrackNode(Node):
    def __init__(self):
        super().__init__('ekf_single_track_node')
        # Subscriber สำหรับ Fake GPS และ Odom ของโมเดล Single Track
        self.create_subscription(Odometry, '/fake_gps', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom_single_track', self.odom_callback, 10)
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_single_track', 10)

        # State vector: [x, y, theta, v]
        self.x = np.zeros((4,1))
        self.P = np.eye(4)
        self.Q = np.diag([0.1, 0.1, 0.05, 0.1])
        self.R = np.diag([0.5, 0.5])

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        # control input: [v, δ] โดย δ คือมุมเลี้ยว (steering angle)
        self.u = np.zeros((2,1))
        self.gps_measurement = None
        self.L = 2.0  # wheelbase (ระยะระหว่างล้อหน้าและหลัง)

    def odom_callback(self, msg):
        # ดึงค่าความเร็ว v และคำนวณมุมเลี้ยว δ
        v = msg.twist.twist.linear.x
        # สมมุติว่า twist.angular.z = (v/L)*tan(δ)
        if abs(v) > 1e-3:
            delta = math.atan((msg.twist.twist.angular.z * self.L) / v)
        else:
            delta = 0.0
        self.u = np.array([[v], [delta]])

    def gps_callback(self, msg):
        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y
        self.gps_measurement = np.array([[gps_x], [gps_y]])

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.1
        self.last_time = current_time

        # ======= Predict =======
        theta = self.x[2,0]
        v = self.x[3,0]
        delta = self.u[1,0]
        # Motion model แบบ Bicycle Model
        x_pred = np.zeros((4,1))
        x_pred[0,0] = self.x[0,0] + v * np.cos(theta) * dt
        x_pred[1,0] = self.x[1,0] + v * np.sin(theta) * dt
        x_pred[2,0] = self.x[2,0] + (v / self.L) * np.tan(delta) * dt
        x_pred[3,0] = v

        # Jacobian ของ motion model (F)
        F = np.array([
            [1, 0, -v*np.sin(theta)*dt, np.cos(theta)*dt],
            [0, 1,  v*np.cos(theta)*dt, np.sin(theta)*dt],
            [0, 0, 1, (1/self.L)*np.tan(delta)*dt],
            [0, 0, 0, 1]
        ])
        P_pred = F @ self.P @ F.T + self.Q

        # ======= Update =======
        if self.gps_measurement is not None:
            H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])
            z_pred = np.array([
                [x_pred[0,0]],
                [x_pred[1,0]]
            ])
            y_err = self.gps_measurement - z_pred
            S = H @ P_pred @ H.T + self.R
            K = P_pred @ H.T @ np.linalg.inv(S)
            self.x = x_pred + K @ y_err
            self.P = (np.eye(4) - K @ H) @ P_pred
        else:
            self.x = x_pred
            self.P = P_pred

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = float(self.x[0,0])
        odom_msg.pose.pose.position.y = float(self.x[1,0])
        q = quaternion_from_euler(0, 0, self.x[2,0])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.ekf_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSingleTrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
