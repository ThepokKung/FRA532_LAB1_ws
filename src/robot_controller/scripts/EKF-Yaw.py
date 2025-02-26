#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_from_euler

class EKFYawRateNode(Node):
    def __init__(self):
        super().__init__('ekf_yaw_rate_node')
        # Subscriber สำหรับ Fake GPS และ Odom ของโมเดล Yaw Rate
        self.create_subscription(Odometry, '/fake_gps', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom_yaw_rate', self.odom_callback, 10)
        # Publisher สำหรับส่งค่า EKF ที่ประมวลผลแล้ว
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_yaw_rate', 10)

        # กำหนดค่าเริ่มต้นของ State vector: [x, y, theta, v]
        self.x = np.zeros((4,1))
        # Covariance matrix เริ่มต้น
        self.P = np.eye(4)
        # Process noise covariance Q (ปรับได้ตามความไม่แน่นอนใน motion)
        self.Q = np.diag([0.1, 0.1, 0.05, 0.1])
        # Measurement noise covariance R (ความไม่แน่นอนของ GPS)
        self.R = np.diag([0.5, 0.5])

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        # เก็บ control input จาก /odom_yaw_rate: [v, yaw_rate]
        self.u = np.zeros((2,1))
        # ตัวแปรเก็บ GPS measurement เมื่อได้รับข้อมูล
        self.gps_measurement = None

    def odom_callback(self, msg):
        # ดึงค่าความเร็ว (v) และ yaw_rate จาก twist ของ odom
        v = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z
        self.u = np.array([[v], [yaw_rate]])

    def gps_callback(self, msg):
        # ดึงตำแหน่งจาก Fake GPS
        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y
        self.gps_measurement = np.array([[gps_x], [gps_y]])

    def timer_callback(self):
        # คำนวณช่วงเวลา dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.1
        self.last_time = current_time

        # ======= ขั้นตอน Predict =======
        theta = self.x[2,0]
        v = self.x[3,0]
        yaw_rate = self.u[1,0]
        # สมการ motion model
        x_pred = np.zeros((4,1))
        x_pred[0,0] = self.x[0,0] + v * np.cos(theta) * dt
        x_pred[1,0] = self.x[1,0] + v * np.sin(theta) * dt
        x_pred[2,0] = self.x[2,0] + yaw_rate * dt
        x_pred[3,0] = v

        # คำนวณ Jacobian ของ motion model (F)
        F = np.array([
            [1, 0, -v*np.sin(theta)*dt, np.cos(theta)*dt],
            [0, 1,  v*np.cos(theta)*dt, np.sin(theta)*dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # คำนวณ Predicted covariance
        P_pred = F @ self.P @ F.T + self.Q

        # ======= ขั้นตอน Update =======
        if self.gps_measurement is not None:
            # Measurement model: วัดเฉพาะตำแหน่ง [x, y]
            H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])
            z_pred = np.array([
                [x_pred[0,0]],
                [x_pred[1,0]]
            ])
            # คำนวณ innovation error
            y_err = self.gps_measurement - z_pred
            S = H @ P_pred @ H.T + self.R
            K = P_pred @ H.T @ np.linalg.inv(S)
            # ปรับปรุง state และ covariance
            self.x = x_pred + K @ y_err
            self.P = (np.eye(4) - K @ H) @ P_pred
        else:
            self.x = x_pred
            self.P = P_pred

        # ส่งข้อมูล EKF ที่ประมวลผลแล้วออกไปในรูปแบบ Odometry
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
    node = EKFYawRateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
