#!/usr/bin/python3
"""
path_tracking_mpc.py
MPC Controller for Path Tracking using Bicycle Model
- โหลด path จาก /Path/path.yaml
- รับตำแหน่งจาก /ground_truth/pose
- ตั้ง horizon และ time step สำหรับ MPC
- แก้ปัญหา optimization เพื่อหาคำสั่ง control (acceleration, steering angle)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import yaml
import numpy as np
import cvxpy as cp
import math
import os

class MPCPathTracking(Node):
    def __init__(self):
        super().__init__('mpc_path_tracking')
        # โหลด path จาก YAML
        path_file = '/Path/path.yaml'
        if os.path.exists(path_file):
            with open(path_file, 'r') as f:
                self.path = yaml.safe_load(f)
        else:
            self.get_logger().error("Path file not found: " + path_file)
            self.path = []
        
        self.declare_parameter('horizon', 10)
        self.N = self.get_parameter('horizon').value  # prediction horizon
        self.dt = 0.1  # time step (s)
        
        # Vehicle parameter: wheelbase (for Bicycle Model)
        self.declare_parameter('wheelbase', 0.10)
        self.L = self.get_parameter('wheelbase').value
        
        # Initialize state: [x, y, theta, v]
        # สามารถปรับค่าเริ่มต้นได้
        self.x0 = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Subscriber for ground truth pose
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.pose_callback, 10)
        # Publisher for cmd_vel output
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
    
    def pose_callback(self, msg: PoseStamped):
        # อ่านตำแหน่งปัจจุบันจาก ground truth
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        q = msg.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y+q.z*q.z))
        # ใช้ velocity จาก state ที่เก็บไว้
        v0 = self.x0[3]
        self.x0 = np.array([cx, cy, theta, v0])
        
        # สำหรับตัวอย่างนี้, สมมติ target เป็น waypoint สุดท้ายของ path
        if len(self.path) == 0:
            target = np.array([cx, cy, theta, v0])
        else:
            target_wp = self.path[-1]
            target = np.array([target_wp['x'], target_wp['y'], target_wp['yaw'], v0])
        
        N = self.N
        dt = self.dt
        
        # Define optimization variables:
        x = cp.Variable((4, N+1))
        u = cp.Variable((2, N))  # u: [acceleration, steering_angle]
        
        cost = 0
        constraints = [x[:,0] == self.x0]
        for k in range(N):
            # Bicycle model dynamics:
            # x_{k+1} = x_k + v_k*cos(theta_k)*dt
            # y_{k+1} = y_k + v_k*sin(theta_k)*dt
            # theta_{k+1} = theta_k + v_k/L * tan(u[1,k])*dt
            # v_{k+1} = v_k + u[0,k]*dt
            constraints += [
                x[0,k+1] == x[0,k] + x[3,k]*cp.cos(x[2,k])*dt,
                x[1,k+1] == x[1,k] + x[3,k]*cp.sin(x[2,k])*dt,
                x[2,k+1] == x[2,k] + (x[3,k]/self.L)*cp.tan(u[1,k])*dt,
                x[3,k+1] == x[3,k] + u[0,k]*dt
            ]
            # Cost: quadratic error to target state and control effort
            cost += cp.sum_squares(x[:,k] - target) + cp.sum_squares(u[:,k])
        
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)
        
        if prob.status == cp.OPTIMAL:
            accel_cmd = u.value[0,0]
            steer_cmd = u.value[1,0]
            cmd = Twist()
            # Use first control input for cmd_vel
            cmd.linear.x = self.x0[3] + accel_cmd*dt
            cmd.angular.z = steer_cmd
            self.cmd_pub.publish(cmd)
            # Update state estimate to next step
            self.x0 = x.value[:,1]
        else:
            self.get_logger().warn("MPC optimization did not converge")
        
        self.get_logger().info(f"MPC: target={target}, control: accel={accel_cmd:.3f}, steer={steer_cmd:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCPathTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
