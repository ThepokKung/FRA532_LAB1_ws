#!/usr/bin/env python3
"""
JointStateForwardKinematicsAll - Forward Kinematics from JointState and IMU using 3 models:
1. Yaw-Rate Model (IMU-based)
2. Single-Track (Bicycle)
3. Double-Track (Differential drive approx)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import tf_transformations


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class JointStateForwardKinematicsAll(Node):
    def __init__(self):
        super().__init__('joint_state_forward_kinematics_all')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('track_width', 0.13)

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value

        # Initialize states
        self.x = {'yaw':0.0, 'single':0.0, 'double':0.0}
        self.y = {'yaw':0.0, 'single':0.0, 'double':0.0}
        self.theta = {'yaw':0.0, 'single':0.0, 'double':0.0}

        # Inputs
        self.v_rl = self.v_rr = 0.0
        self.delta_left = self.delta_right = 0.0
        self.yaw_rate = 0.0

        # Commands stored for integration
        self.v_cmd = {'yaw':0.0, 'single':0.0, 'double':0.0}
        self.w_cmd = {'yaw':0.0, 'single':0.0, 'double':0.0}

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.jointstate_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publishers
        self.pub_yaw    = self.create_publisher(Odometry, '/odom_yaw_rate',    10)
        self.pub_single = self.create_publisher(Odometry, '/odom_single_track',10)
        self.pub_double = self.create_publisher(Odometry, '/odom_double_track',10)

        # Timer
        self.dt = 0.01
        self.create_timer(self.dt, self.update_kinematics)

        self.get_logger().info('JointStateForwardKinematicsAll initialized.')

    def jointstate_callback(self, msg: JointState):
        idx_rl = idx_rr = idx_dl = idx_dr = None
        for i, name in enumerate(msg.name):
            if name=='back_left_wheel':    idx_rl = i
            elif name=='back_right_wheel': idx_rr = i
            elif name=='front_left_steering_joint':  idx_dl = i
            elif name=='front_right_steering_joint': idx_dr = i
        if None in (idx_rl, idx_rr, idx_dl, idx_dr):
            self.get_logger().warn('Missing joints')
            return
        # compute rear-wheel speeds
        self.v_rl = msg.velocity[idx_rl] * self.r
        self.v_rr = msg.velocity[idx_rr] * self.r
        # steering angles
        self.delta_left  = msg.position[idx_dl]
        self.delta_right = msg.position[idx_dr]

    def imu_callback(self, msg: Imu):
        # store IMU yaw rate
        self.yaw_rate = msg.angular_velocity.z

    def update_kinematics(self):
        # compute average v and slip-angle
        V = (self.v_rl + self.v_rr)/2.0
        delta = 0.5*(self.delta_left + self.delta_right)

        # 1) Yaw-Rate Model
        # integrate previous command
        mid = self.theta['yaw'] + 0.5*self.w_cmd['yaw']*self.dt
        self.x['yaw']     += self.v_cmd['yaw']*math.cos(mid)*self.dt
        self.y['yaw']     += self.v_cmd['yaw']*math.sin(mid)*self.dt
        self.theta['yaw'] += self.w_cmd['yaw']*self.dt
        # update commands
        self.v_cmd['yaw'] = V
        self.w_cmd['yaw'] = self.yaw_rate
        # publish
        odom = self.make_odom(self.x['yaw'], self.y['yaw'], self.theta['yaw'],
                              self.v_cmd['yaw'], self.w_cmd['yaw'])
        self.pub_yaw.publish(odom)

        # 2) Single-Track Model
        mid = self.theta['single'] + 0.5*self.w_cmd['single']*self.dt
        self.x['single']     += self.v_cmd['single']*math.cos(mid)*self.dt
        self.y['single']     += self.v_cmd['single']*math.sin(mid)*self.dt
        self.theta['single'] += self.w_cmd['single']*self.dt
        # update commands
        self.v_cmd['single'] = V
        self.w_cmd['single'] = (V/self.L)*math.tan(delta)
        odom = self.make_odom(self.x['single'], self.y['single'], self.theta['single'],
                              self.v_cmd['single'], self.w_cmd['single'])
        self.pub_single.publish(odom)

        # 3) Double-Track Model
        mid = self.theta['double'] + 0.5*self.w_cmd['double']*self.dt
        self.x['double']     += self.v_cmd['double']*math.cos(mid)*self.dt
        self.y['double']     += self.v_cmd['double']*math.sin(mid)*self.dt
        self.theta['double'] += self.w_cmd['double']*self.dt
        # update commands
        self.v_cmd['double'] = V
        # wheel difference
        self.w_cmd['double'] = (self.v_rr - self.v_rl)/self.W
        odom = self.make_odom(self.x['double'], self.y['double'], self.theta['double'],
                              self.v_cmd['double'], self.w_cmd['double'])
        self.pub_double.publish(odom)

    def make_odom(self, x, y, theta, v, w):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0,0,theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        return odom


def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardKinematicsAll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
