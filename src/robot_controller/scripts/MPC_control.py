#!/usr/bin/python3
"""
mpc_controller.py

MPC Controller for Ackermann Steering Mobile Robot.

- Load path from YAML file (from package 'robot_controller/config/path.yaml')
- Subscribe to ground truth odometry from /ground_truth/odom
- Continuously follow the path from the first waypoint to the final waypoint.
- Use MPC to compute control commands.
- Publish cmd_vel (Twist message) on /cmd_vel (linear.x and angular.z)
- Detailed logging is provided at initialization and during each control cycle.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.optimize import minimize

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # Load path from YAML file
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        try:
            with open(path_file, 'r') as f:
                self.path = yaml.safe_load(f)
            self.get_logger().info(
                f"✅ Loaded path with {len(self.path)} waypoints")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load path: {e}")
            self.path = []

        # MPC parameters
        self.declare_parameter('horizon', 10)  # Prediction horizon
        self.declare_parameter('dt', 0.1)      # Time step
        self.declare_parameter('Q', [1.0, 1.0, 0.1])  # State cost weights
        self.declare_parameter('R', [0.1, 0.1])      # Control cost weights
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.Q = np.diag(self.get_parameter('Q').value)
        self.R = np.diag(self.get_parameter('R').value)

        # Vehicle parameter: wheelbase L (m)
        self.declare_parameter('wheelbase', 0.20)
        self.L = self.get_parameter('wheelbase').value

        # Control parameters
        self.declare_parameter('linear_velocity', 5.0)  # m/s
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # Threshold for waypoint reaching
        self.declare_parameter('waypoint_threshold', 1.0)  # m
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value

        # Current target index
        self.current_target_index = 0

        # Subscriber for current pose (Odometry) from ground truth or state estimator
        self.create_subscription(
            Odometry, '/ground_truth/pose', self.odom_callback, 10)
        # Publisher for cmd_vel command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.prev_time = None

        self.get_logger().info("🚀 MPC Controller Node Initialized")
        self.get_logger().info(f"🔹 Horizon: {self.horizon}, dt: {self.dt}")
        self.get_logger().info(f"🔹 Q: {self.Q}, R: {self.R}")
        self.get_logger().info(
            f"🔹 Wheelbase: {self.L} m, Linear Velocity: {self.linear_velocity} m/s")

    def odom_callback(self, msg: Odometry):
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        current_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        v = msg.twist.twist.linear.x

        # Get current target waypoint
        if self.current_target_index >= len(self.path):
            self.get_logger().info("🏁 Final waypoint reached. Stopping.")
            self.publish_stop()
            return
        target_wp = self.path[self.current_target_index]
        tx = target_wp['x']
        ty = target_wp['y']

        distance_error = math.hypot(tx - cx, ty - cy)
        if distance_error < self.waypoint_threshold:
            self.get_logger().info(
                f"✅ Reached waypoint {self.current_target_index} (error={distance_error:.2f} m)")
            self.current_target_index += 1
            if self.current_target_index >= len(self.path):
                self.get_logger().info("🏁 Final waypoint reached. Stopping.")
                self.publish_stop()
                return
            target_wp = self.path[self.current_target_index]
            tx = target_wp['x']
            ty = target_wp['y']

        desired_heading = math.atan2(ty - cy, tx - cx)
        heading_error = normalize_angle(desired_heading - current_yaw)

        # Implement MPC to compute control commands
        u = self.solve_mpc(cx, cy, current_yaw, v, tx, ty, heading_error)

        # Convert steering angle to angular velocity
        angular_velocity = u[0] * math.tan(u[1]) / self.L

        # Log debug info
        self.get_logger().info(
            f"📍 Pose: x={cx:.2f}, y={cy:.2f}, yaw={math.degrees(current_yaw):.1f}°")
        self.get_logger().info(
            f"🎯 Target[{self.current_target_index}]: x={tx:.2f}, y={ty:.2f}, Dist Err: {distance_error:.2f} m")
        self.get_logger().info(
            f"   Desired Heading: {math.degrees(desired_heading):.1f}°, Heading Error: {math.degrees(heading_error):.1f}°")
        self.get_logger().info(
            f"   Control Commands: v={u[0]:.2f} m/s, δ={math.degrees(u[1]):.1f}° (ω={angular_velocity:.2f} rad/s)")

        # Create and publish cmd_vel message
        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = angular_velocity
        self.cmd_pub.publish(cmd)

    def solve_mpc(self, cx, cy, current_yaw, v, tx, ty, heading_error):
        # Define the cost function for MPC
        def cost_function(u):
            cost = 0.0
            x = cx
            y = cy
            yaw = current_yaw
            for i in range(self.horizon):
                x += u[0] * math.cos(yaw) * self.dt
                y += u[0] * math.sin(yaw) * self.dt
                yaw += u[1] * self.dt
                state_error = np.array(
                    [x - tx, y - ty, normalize_angle(yaw - heading_error)])
                control_error = np.array([self.linear_velocity - u[0], u[1]])
                cost += state_error.T @ self.Q @ state_error + \
                    control_error.T @ self.R @ control_error
            return cost

        # Initial guess for control inputs
        u0 = np.array([self.linear_velocity, 0.0])

        # Solve the optimization problem
        result = minimize(cost_function, u0, bounds=[
                          (0, self.linear_velocity), (-np.pi/4, np.pi/4)])
        return result.x

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("🛑 Robot Stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
