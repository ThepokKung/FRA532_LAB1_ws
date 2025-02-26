#!/usr/bin/env python3
"""
JointStateForwardKinematicsAll - Forward Kinematics from JointState using 3 Models:
1. Yaw Rate Model (Differential Drive)
2. Single-Track (Bicycle) Model
3. Double-Track Model

หมายเหตุ:
- ข้อมูลเข้ามาจาก topic /joint_states ซึ่งมีข้อมูลของ wheel velocities และ steering joint positions
- ค่าพารามิเตอร์ (wheel_radius, wheelbase, track_width) ควรตรวจสอบให้ตรงกับรถของคุณ
- สำหรับ Double-Track Model ค่าต่างๆ (d_rr, d_rl, dr) จะถูกคำนวณโดยใช้ wheelbase และ track_width
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import tf_transformations

class JointStateForwardKinematicsAll(Node):
    def __init__(self):
        super().__init__('joint_state_forward_kinematics_all')

        # กำหนดพารามิเตอร์ของหุ่นยนต์
        self.declare_parameter('wheel_radius', 0.045)    # หน่วย: เมตร
        self.declare_parameter('wheelbase', 0.20)          # ระยะห่างล้อหน้า-หลัง (เมตร)
        self.declare_parameter('track_width', 0.13)          # ระยะห่างล้อซ้าย-ขวา (เมตร)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

        # สำหรับ Double-Track Model คำนวณค่าตัวแปรจากพารามิเตอร์ที่ให้มา
        # d_rr: มุมของล้อขวา, d_rl: มุมของล้อซ้าย, dr: ระยะห่างระหว่าง contact points
        self.d_rr = math.atan(self.L / self.W)        # ใช้ L/W แทน 0.20/0.13
        self.d_rl = math.pi - self.d_rr
        self.dr = math.sqrt(self.L**2 + self.W**2)

        # สถานะเริ่มต้นของ odometry สำหรับแต่ละโมเดล
        # --- Yaw Rate Model ---
        self.x_yaw = 0.0
        self.y_yaw = 0.0
        self.theta_yaw = 0.0
        self.v_yaw = 0.0
        self.omega_yaw = 0.0

        # --- Single-Track (Bicycle) Model ---
        self.x_single = 0.0
        self.y_single = 0.0
        self.theta_single = 0.0
        self.v_single = 0.0
        self.omega_single = 0.0

        # --- Double-Track Model ---
        self.x_double = 0.0
        self.y_double = 0.0
        self.theta_double = 0.0
        self.v_double = 0.0
        self.omega_double = 0.0

        self.prev_time = None

        # Subscriber สำหรับ JointState
        self.create_subscription(JointState, '/joint_states', self.jointstate_callback, 10)

        # Publisher สำหรับ odometry ของแต่ละโมเดล
        self.odom_pub_yaw = self.create_publisher(Odometry, '/odom_yaw_rate', 10)
        self.odom_pub_single = self.create_publisher(Odometry, '/odom_single_track', 10)
        self.odom_pub_double = self.create_publisher(Odometry, '/odom_double_track', 10)

        self.get_logger().info("JointState Forward Kinematics All node has started.")

    def jointstate_callback(self, msg: JointState):
        # สร้าง dictionary เพื่อเข้าถึงข้อมูล joint ได้ง่าย
        joints = {name: {'position': pos, 'velocity': vel} 
                  for name, pos, vel in zip(msg.name, msg.position, msg.velocity)}

        # ตรวจสอบว่ามี joint ที่จำเป็นครบถ้วนหรือไม่
        required = [
            "front_left_wheel", "front_right_wheel",
            "back_left_wheel", "back_right_wheel",
            "front_left_steering_joint", "front_right_steering_joint"
        ]
        for joint in required:
            if joint not in joints:
                self.get_logger().warn(f"Missing joint {joint} in JointState")
                return

        # คำนวณความเร็วของล้อ (m/s) โดยใช้ wheel_radius
        # สำหรับ odometry จะใช้ข้อมูลจากล้อหลังเท่านั้น
        v_rl = joints["back_left_wheel"]['velocity'] * self.wheel_radius
        v_rr = joints["back_right_wheel"]['velocity'] * self.wheel_radius

        # รับค่า steering (มุมเลี้ยว) จาก front steering joints (rad)
        delta_left = joints["front_left_steering_joint"]['position']
        delta_right = joints["front_right_steering_joint"]['position']
        # ค่า steering โดยเฉลี่ยจากล้อหน้า
        delta = (delta_left + delta_right) / 2.0

        # คำนวณค่า dt จาก header.stamp ของ JointState message
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # หาก dt มีค่าน้อยเกินไป ให้ข้ามการอัปเดตเพื่อป้องกันการคำนวณที่ผิดพลาด
        if dt < 1e-6:
            return

        # ==============================
        # 1. Yaw Rate Model (Differential Drive)
        # ==============================
        # ใช้ล้อหลังทั้งสองคำนวณความเร็วเชิงเส้นและอัตราการหมุน
        v_left_yaw = v_rl
        v_right_yaw = v_rr
        V_yaw = (v_left_yaw + v_right_yaw) / 2.0
        omega_yaw = (v_right_yaw - v_left_yaw) / self.W

        # อัปเดต state โดยใช้ mid-point integration
        self.x_yaw += V_yaw * dt * math.cos(self.theta_yaw + (omega_yaw * dt / 2.0))
        self.y_yaw += V_yaw * dt * math.sin(self.theta_yaw + (omega_yaw * dt / 2.0))
        self.theta_yaw += omega_yaw * dt

        self.v_yaw = V_yaw
        self.omega_yaw = omega_yaw

        odom_yaw = self.create_odom_msg(self.x_yaw, self.y_yaw, self.theta_yaw,V_yaw, omega_yaw, msg.header.stamp)

        # ==============================
        # 2. Single-Track (Bicycle) Model
        # ==============================
        # ใช้ค่าเฉลี่ยของล้อหลังเป็นความเร็วเชิงเส้น
        V_single = (v_rl + v_rr) / 2.0
        # คำนวณอัตราการหมุน: ω = (V / L) * tan(δ)
        omega_single = (V_single / self.L) * math.tan(delta)
        # อัปเดต state ด้วย mid-point integration
        self.x_single += V_single * dt * math.cos(self.theta_single + (omega_single * dt / 2.0))
        self.y_single += V_single * dt * math.sin(self.theta_single + (omega_single * dt / 2.0))
        self.theta_single += omega_single * dt

        self.v_single = V_single
        self.omega_single = omega_single

        odom_single = self.create_odom_msg(self.x_single, self.y_single, self.theta_single,
                                            V_single, omega_single, msg.header.stamp)

        # ==============================
        # 3. Double-Track Model
        # ==============================
        # ใช้ล้อหลังในการคำนวณความเร็วเชิงเส้น
        V_double = (v_rl + v_rr) / 2.0
        # คำนวณอัตราการหมุนตามสูตร:
        # ω = (v_rr - v_rl) / [ dr * ( sin(d_rl + θ) - sin(d_rr + θ) ) ]
        denom = self.dr * (math.sin(self.d_rl + self.theta_double) - math.sin(self.d_rr + self.theta_double))
        if abs(denom) < 1e-6:
            omega_double = 0.0
        else:
            omega_double = (v_rr - v_rl) / denom

        self.x_double += V_double * dt * math.cos(self.theta_double + (omega_double * dt / 2.0))
        self.y_double += V_double * dt * math.sin(self.theta_double + (omega_double * dt / 2.0))
        self.theta_double += omega_double * dt

        self.v_double = V_double
        self.omega_double = omega_double

        odom_double = self.create_odom_msg(self.x_double, self.y_double, self.theta_double,V_double, omega_double, msg.header.stamp)

        # Publish Odometry messages สำหรับแต่ละโมเดล
        self.odom_pub_yaw.publish(odom_yaw)
        self.odom_pub_single.publish(odom_single)
        self.odom_pub_double.publish(odom_double)

    def create_odom_msg(self, x, y, theta, V, omega, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = V
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        return odom

def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardKinematicsAll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
