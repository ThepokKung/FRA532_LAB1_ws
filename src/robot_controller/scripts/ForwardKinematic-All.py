#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

class JointStateForwardKinematicsAll(Node):
    def __init__(self):
        super().__init__('joint_state_forward_kinematics_all')

        # พารามิเตอร์ของหุ่นยนต์
        self.declare_parameter('wheel_radius', 0.045)    # m
        self.declare_parameter('wheelbase', 0.10)          # m (ระยะห่างระหว่างล้อหน้า-หลัง)
        self.declare_parameter('track_width', 0.13)          # m (ระยะห่างระหว่างล้อซ้าย-ขวา)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

        # สถานะเริ่มต้นของ odometry
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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("JointState Forward Kinematics All node has started.")

    def jointstate_callback(self, msg: JointState):
        # สร้าง dictionary สำหรับเข้าถึงข้อมูลได้ง่าย
        joints = {name: {'position': pos, 'velocity': vel} for name, pos, vel in zip(msg.name, msg.position, msg.velocity)}

        # ตรวจสอบข้อมูลที่จำเป็น (ชื่อของ joints ควรตรงกับใน URDF ของคุณ)
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
        v_fl = joints["front_left_wheel"]['velocity'] * self.wheel_radius
        v_fr = joints["front_right_wheel"]['velocity'] * self.wheel_radius
        v_rl = joints["back_left_wheel"]['velocity'] * self.wheel_radius
        v_rr = joints["back_right_wheel"]['velocity'] * self.wheel_radius

        # รับค่า steering (มุมเลี้ยว) จาก steering joints (rad)
        # สมมติใช้ค่าเฉลี่ยของล้อหน้า
        delta_left = joints["front_left_steering_joint"]['position']
        delta_right = joints["front_right_steering_joint"]['position']
        delta = (delta_left + delta_right) / 2.0

        # คำนวณเวลาที่ผ่านมา
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = curr_time - self.prev_time
        self.prev_time = curr_time
        
        # ### 1. Yaw Rate Model (Differential Drive)
        self.x_yaw += self.v_yaw * dt * math.cos(self.theta_yaw + ((self.omega_yaw * dt )/ 2.0))
        self.y_yaw += self.v_yaw * dt * math.sin(self.theta_yaw + ((self.omega_yaw * dt )/ 2.0))
        self.theta_yaw += self.omega_yaw * dt
        self.v_yaw = (v_rl + v_rr) / 2.0

        self.omega_yaw = (v_rr - v_rl) / self.W

        odom_yaw = self.create_odom_msg(self.x_yaw, self.y_yaw, self.theta_yaw, self.v_yaw, self.omega_yaw, msg.header.stamp)

        # ### 2. Single-Track (Bicycle) Model
        self.x_single += self.v_single * dt * math.cos(self.theta_single + ((self.omega_single * dt )/ 2.0))
        self.y_single += self.v_single * dt * math.sin(self.theta_single + ((self.omega_single * dt )/ 2.0))
        self.theta_single += self.omega_single * dt
    
        self.omega_single = self.v_single / self.L * math.tan(delta)
        self.v_single = (v_rl + v_rr) / 2.0

        odom_single = self.create_odom_msg(self.x_single, self.y_single, self.theta_single, self.v_single , self.omega_single , msg.header.stamp)

        ### 3. Double-Track Model
        d_rr = math.atan(10.0 / 6.5)
        d_rl = 180 - d_rr
        dr = math.sqrt(10.0**2 + 6.5**2)

        self.x_double += self.v_double * dt * math.cos(self.theta_double + ((self.omega_double * dt )/ 2.0))
        self.y_double += self.v_double * dt * math.sin(self.theta_double + ((self.omega_double * dt )/ 2.0))
        self.theta_double += self.omega_double * dt
        self.v_double = (v_rl + v_rr) / 2.0

        self.omega_double = (v_rr - v_rl) / ((self.y_double - (dr * math.sin(d_rr + self.theta_double))) - (self.y_double - (dr * math.sin(d_rl + self.theta_double))))

        odom_double = self.create_odom_msg(self.x_double, self.y_double, self.theta_double, self.v_double , self.omega_double, msg.header.stamp)

        # Publish Odometry messages
        self.odom_pub_yaw.publish(odom_yaw)
        self.odom_pub_single.publish(odom_single)
        self.odom_pub_double.publish(odom_double)

        # Broadcast the transform
        self.broadcast_transform(self.x_yaw, self.y_yaw, self.theta_yaw, msg.header.stamp)

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
        q = quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear.x = V
        odom.twist.twist.angular.z = omega
        return odom

    def broadcast_transform(self, x, y, theta, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardKinematicsAll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()