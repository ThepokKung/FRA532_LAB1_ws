#!/usr/bin/python3
"""
JointStateForwardKinematicsAll - Forward Kinematics from JointState using 3 Models:
1. Yaw Rate Model (Differential Drive)
2. Single-Track (Bicycle) Model
3. Double-Track Model

หมายเหตุ:
- ข้อมูลเข้ามาจาก topic /joint_states ซึ่งมีข้อมูลของ wheel velocities และ steering joint positions
- ค่าพารามิเตอร์ (wheel_radius, wheelbase, track_width) ควรตรวจสอบให้ตรงกับรถของคุณ
- ในโมเดล Double-Track, ค่าคงที่ 10.0 และ 6.5 ถูกใช้เป็นตัวอย่างจาก Paper; หากมีข้อมูลจริงให้ปรับเปลี่ยนให้ถูกต้อง
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations

class JointStateForwardKinematicsAll(Node):
    def __init__(self):
        super().__init__('joint_state_forward_kinematics_all')

        # พารามิเตอร์ของหุ่นยนต์
        self.declare_parameter('wheel_radius', 0.045)    # m
        self.declare_parameter('wheelbase', 0.20)          # m (ระยะห่างล้อหน้า-หลัง)
        self.declare_parameter('track_width', 0.13)          # m (ระยะห่างล้อซ้าย-ขวา)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

        # สถานะเริ่มต้นของ odometry สำหรับแต่ละโมเดล
        self.x_yaw = 0.0
        self.y_yaw = 0.0
        self.theta_yaw = 0.0
        self.v_yaw = 0.0
        self.omega_yaw = 0.0

        self.x_single = 0.0
        self.y_single = 0.0
        self.theta_single = 0.0
        self.v_single = 0.0
        self.omega_single = 0.0

        self.x_double = 0.0
        self.y_double = 0.0
        self.theta_double = 0.0
        self.v_double = 0.0
        self.omega_double = 0.0

        self.prev_time = None

        # Subscriber JointState
        self.create_subscription(JointState, '/joint_states', self.jointstate_callback, 10)

        # Publishers สำหรับ odometry ของแต่ละโมเดล
        self.odom_pub_yaw = self.create_publisher(Odometry, '/odom_yaw_rate', 10)
        self.odom_pub_single = self.create_publisher(Odometry, '/odom_single_track', 10)
        self.odom_pub_double = self.create_publisher(Odometry, '/odom_double_track', 10)

        # Transform broadcaster
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("JointState Forward Kinematics All node has started.")

    def jointstate_callback(self, msg: JointState):
        # สร้าง dictionary สำหรับเข้าถึงข้อมูลได้ง่าย
        joints = {name: {'position': pos, 'velocity': vel} for name, pos, vel in zip(msg.name, msg.position, msg.velocity)}

        # ตรวจสอบข้อมูลที่จำเป็น
        required = [
            "front_left_wheel", "front_right_wheel",
            "back_left_wheel", "back_right_wheel",
            "front_left_steering_joint", "front_right_steering_joint"
        ]
        for joint in required:
            if joint not in joints:
                self.get_logger().warn(f"Missing joint {joint} in JointState")
                return

        # คำนวณความเร็วของล้อ (m/s) จาก wheel joint velocities
        # v_fl and v_fr are not used, so they are removed
        v_rl = joints["back_left_wheel"]['velocity'] * self.wheel_radius
        v_rr = joints["back_right_wheel"]['velocity'] * self.wheel_radius

        # รับค่า steering (มุมเลี้ยว) จาก steering joints (rad)
        delta_left = joints["front_left_steering_joint"]['position']
        delta_right = joints["front_right_steering_joint"]['position']
        # ใน Paper, ค่า steering ถูกนิยามจากค่าเฉลี่ยของล้อหน้า
        delta = (delta_left + delta_right) / 2.0

        # คำนวณเวลาที่ผ่านมา
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # ======= 1. Yaw Rate Model =======
        # ใช้ล้อหลังในการคำนวณ (Differential Drive)
        # คำนวณความเร็วเชิงเส้นและอัตราการหมุน
        v_left_yaw = v_rl #C
        v_right_yaw = v_rr #C
        V_yaw = (v_left_yaw + v_right_yaw) / 2.0 #C
        omega_yaw = (v_right_yaw - v_left_yaw) / self.W #C

        # อัปเดต state โดยใช้ mid-point integration
        self.x_yaw += V_yaw * dt * math.cos(self.theta_yaw + (omega_yaw * dt / 2.0)) #C
        self.y_yaw += V_yaw * dt * math.sin(self.theta_yaw + (omega_yaw * dt / 2.0)) #C
        self.theta_yaw += omega_yaw * dt #C

        self.v_yaw = V_yaw #C
        self.omega_yaw = omega_yaw #C

        odom_yaw = self.create_odom_msg(self.x_yaw, self.y_yaw, self.theta_yaw, V_yaw, omega_yaw, msg.header.stamp)

        # ======= 2. Single-Track (Bicycle) Model =======
        # ใช้ค่าเฉลี่ยของล้อหลังเป็นความเร็วเชิงเส้น
        V_single = (v_rl + v_rr) / 2.0
        # คำนวณอัตราการหมุนจาก Bicycle Model: ω = V/L * tan(δ)
        omega_single = V_single / self.L * math.tan(delta)
        # อัปเดต state
        self.x_single += V_single * dt * math.cos(self.theta_single + (omega_single * dt / 2.0))
        self.y_single += V_single * dt * math.sin(self.theta_single + (omega_single * dt / 2.0))
        self.theta_single += omega_single * dt

        self.v_single = V_single
        self.omega_single = omega_single

        odom_single = self.create_odom_msg(self.x_single, self.y_single, self.theta_single, V_single, omega_single, msg.header.stamp)

        # ======= 3. Double-Track Model =======
        d_rr = math.atan(0.20 / 0.13)    # มุมที่คำนวณจากล้อขวา (ตัวอย่าง)
        d_rl = math.pi - d_rr           # มุมของล้อซ้าย (แก้ไขจาก 180 - d_rr)
        dr = math.sqrt(0.20**2 + 0.13**2)  # ระยะห่างระหว่าง contact points (ตัวอย่าง)
        # การคำนวณความเร็วเชิงเส้นสำหรับ Double-Track Model ใช้ล้อหลังเฉพาะ
        V_double = (v_rl + v_rr) / 2.0

        # คำนวณอัตราการหมุนแบบ Double-Track:
        # ω = (v_rr - v_rl) / (dr * [ sin(d_rl + θ) - sin(d_rr + θ) ])
        denom = dr * (math.sin(d_rl + self.theta_double) - math.sin(d_rr + self.theta_double))
        if abs(denom) < 1e-6:
            omega_double = 0.0
        else:
            omega_double = (v_rr - v_rl) / denom

        # อัปเดต state สำหรับ Double-Track Model
        self.x_double += V_double * dt * math.cos(self.theta_double + (omega_double * dt / 2.0))
        self.y_double += V_double * dt * math.sin(self.theta_double + (omega_double * dt / 2.0))
        self.theta_double += omega_double * dt

        self.v_double = V_double
        self.omega_double = omega_double

        odom_double = self.create_odom_msg(self.x_double, self.y_double, self.theta_double, V_double, omega_double, msg.header.stamp)

        # Publish Odometry messages
        self.odom_pub_yaw.publish(odom_yaw)
        self.odom_pub_single.publish(odom_single)
        self.odom_pub_double.publish(odom_double)

        # Broadcast the transform (ใช้ state ของ Yaw Rate Model เป็นตัวแทน)
        # self.broadcast_transform(self.x_yaw, self.y_yaw, self.theta_yaw, msg.header.stamp)

    def create_odom_msg(self, x, y, theta, V, omega, stamp):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Quaternion
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

    # def broadcast_transform(self, x, y, theta, stamp):
    #     t = TransformStamped()
    #     t.header.stamp = stamp
    #     t.header.frame_id = "odom"
    #     t.child_frame_id = "base_link"
    #     t.transform.translation.x = x
    #     t.transform.translation.y = y
    #     t.transform.translation.z = 0.0
    #     q = tf_transformations.quaternion_from_euler(0, 0, theta)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]
    #     self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardKinematicsAll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
