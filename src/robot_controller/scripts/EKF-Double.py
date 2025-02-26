#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_from_euler

class EKFDoubleTrackNode(Node):
    def __init__(self):
        super().__init__('ekf_double_track_node')
        # Subscriber สำหรับ Fake GPS และ Odom ของโมเดล Double Track
        self.create_subscription(Odometry, '/fake_gps', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom_double_track', self.odom_callback, 10)
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_double_track', 10)

        # State vector: [x, y, theta]
        self.x = np.zeros((3,1))
        self.P = np.eye(3)
        self.Q = np.diag([0.1, 0.1, 0.05])
        self.R = np.diag([0.5, 0.5])

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        # ค่าควบคุมที่ได้รับจาก odom ของ Double Track
        self.v_avg = 0.0      # ความเร็วเฉลี่ย
        self.yaw_rate = 0.0   # อัตราการเปลี่ยนแปลงมุม
        self.L = 1.0          # ระยะห่างระหว่าง track (หรือ wheelbase)

        self.gps_measurement = None

    def odom_callback(self, msg):
        # สมมุติว่า /odom_double_track ให้ค่า linear.x เป็นความเร็วเฉลี่ย
        # และ twist.angular.z เป็น yaw_rate ที่คำนวณจากความแตกต่างของความเร็วขวาและซ้าย
        self.v_avg = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

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
        # Motion model สำหรับ Double Track:
        # x_new = x + v_avg*cos(theta)*dt
        # y_new = y + v_avg*sin(theta)*dt
        # theta_new = theta + yaw_rate*dt
        theta = self.x[2,0]
        x_pred = np.zeros((3,1))
        x_pred[0,0] = self.x[0,0] + self.v_avg * np.cos(theta) * dt
        x_pred[1,0] = self.x[1,0] + self.v_avg * np.sin(theta) * dt
        x_pred[2,0] = self.x[2,0] + self.yaw_rate * dt

        # Jacobian F ของ motion model
        F = np.array([
            [1, 0, -self.v_avg * np.sin(theta) * dt],
            [0, 1,  self.v_avg * np.cos(theta) * dt],
            [0, 0, 1]
        ])
        P_pred = F @ self.P @ F.T + self.Q

        # ======= Update =======
        if self.gps_measurement is not None:
            H = np.array([
                [1, 0, 0],
                [0, 1, 0]
            ])
            z_pred = np.array([
                [x_pred[0,0]],
                [x_pred[1,0]]
            ])
            y_err = self.gps_measurement - z_pred
            S = H @ P_pred @ H.T + self.R
            K = P_pred @ H.T @ np.linalg.inv(S)
            self.x = x_pred + K @ y_err
            self.P = (np.eye(3) - K @ H) @ P_pred
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
    node = EKFDoubleTrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
