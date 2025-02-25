#!/usr/bin/python3
"""
mpc_control.py

MPC Path Tracking Controller for Ackermann Steering Mobile Robot

- Load path from YAML file (from package 'robot_controller/config/path.yaml')
- Subscribe to ground truth odometry from /ground_truth/pose
- Continuously follow the path from the first waypoint to the final waypoint.
- Use MPC to minimize error between desired path and current trajectory.
- When close enough to the target waypoint (within a threshold), move on to the next waypoint.
- Publish cmd_vel (Twist message) on /cmd_vel (linear.x and angular.z).
- This version uses a cost function that includes error in position and yaw (using reference yaw from the waypoint).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
import math
import os
import numpy as np
from scipy.optimize import minimize
from ament_index_python.packages import get_package_share_directory

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class MPCPathTracking(Node):
    def __init__(self):
        super().__init__('mpc_path_tracking')

        # Load path from YAML file in the package
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        if os.path.exists(path_file):
            with open(path_file, 'r') as f:
                self.path = yaml.safe_load(f)
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # MPC parameters
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.1)
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value

        # Lookahead threshold: distance threshold to consider a waypoint reached
        self.declare_parameter('lookahead_threshold', 0.3)
        self.lookahead_threshold = self.get_parameter('lookahead_threshold').value

        # Constant linear velocity (m/s)
        self.declare_parameter('linear_velocity', 10.0)
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # Vehicle parameter: wheelbase (m)
        self.declare_parameter('wheelbase', 0.2)
        self.wheelbase = self.get_parameter('wheelbase').value

        # Cost matrices (tunable)
        # Q: State tracking cost for [x, y, yaw]
        self.Q = np.diag([10.0, 10.0, 50.0])
        # R: Control effort cost for steering (δ)
        self.R = np.diag([0.009])

        # Current target index in the path
        self.current_target_index = 0

        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🚀 MPC Path Tracking Node Initialized")
        self.get_logger().info(f"🔹 Horizon = {self.horizon}")
        self.get_logger().info(f"🔹 Time Step = {self.dt} s")
        self.get_logger().info(f"🔹 Lookahead Threshold = {self.lookahead_threshold} m")
        self.get_logger().info(f"🔹 Linear Velocity = {self.linear_velocity} m/s")

    def odom_callback(self, msg: Odometry):
        # Extract current pose from Odometry message
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.get_logger().info(f"📍 Current Pose: x={cx:.3f}, y={cy:.3f}, yaw={math.degrees(current_yaw):.2f}°")

        # If no path is loaded, do nothing
        if not self.path:
            self.get_logger().warn("No path loaded!")
            return

        # Choose the current target waypoint
        if self.current_target_index >= len(self.path):
            self.get_logger().info("🏁 No more waypoints. Stopping robot.")
            self.publish_stop()
            return
        
        target_wp = self.path[self.current_target_index]
        tx = target_wp['x']
        ty = target_wp['y']
        # Use provided yaw from waypoint as reference yaw
        phi_ref = target_wp.get('yaw', 0.0)

        desired_heading = math.atan2(ty - cy, tx - cx)
        heading_error = normalize_angle(desired_heading - current_yaw)
        distance_error = math.hypot(tx - cx, ty - cy)

        self.get_logger().info(f"🎯 Target Waypoint [{self.current_target_index}]: x={tx:.3f}, y={ty:.3f}")
        self.get_logger().info(f"   Distance Error: {distance_error:.3f} m, Heading Error: {math.degrees(heading_error):.2f}°")

        # If close enough to target, move to the next waypoint
        if distance_error < self.lookahead_threshold:
            self.get_logger().info(f"✅ Reached waypoint {self.current_target_index} (error={distance_error:.2f} m)")
            self.current_target_index += 1
            if self.current_target_index >= len(self.path):
                self.get_logger().info("🏁 Reached final waypoint. Stopping robot.")
                self.publish_stop()
                return
            target_wp = self.path[self.current_target_index]
            tx = target_wp['x']
            ty = target_wp['y']
            phi_ref = target_wp.get('yaw', 0.0)

        # MPC optimization using scipy.optimize.minimize
        # Initial state: [x, y, yaw]
        x0 = np.array([cx, cy, current_yaw])
        # Initial guess for control over horizon (one value per step)
        u0 = np.zeros(self.horizon)
        bounds = [(-np.pi, np.pi) for _ in range(self.horizon)]
        # Pass phi_ref as additional argument
        result = minimize(self.mpc_cost, u0, args=(x0, tx, ty, phi_ref), bounds=bounds, method='SLSQP')

        if result.success:
            self.get_logger().info(f"📍 Current Position: x={cx:.3f}, y={cy:.3f}, yaw={math.degrees(current_yaw):.2f}°")
            optimal_steering_angle = result.x[0]
            self.get_logger().info(f"🔄 MPC: optimal_steering_angle = {math.degrees(optimal_steering_angle):.2f}°")

            # Convert steering angle to angular velocity using bicycle model:
            # ω = v_ref * tan(delta) / wheelbase
            angular_velocity = self.linear_velocity * math.tan(optimal_steering_angle) / self.wheelbase

            # Create and publish cmd_vel message
            cmd = Twist()
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = angular_velocity
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"🚗 Publishing cmd_vel: linear.x={cmd.linear.x:.2f} m/s, angular.z={math.degrees(cmd.angular.z):.2f}°/s")
        else:
            self.get_logger().error("❌ MPC optimization failed.")
            self.publish_stop()

    def mpc_cost(self, u, x0, tx, ty, phi_ref):
        """Cost function for MPC.
           u: control sequence over horizon (steering angles) [array of length horizon]
           x0: initial state [x, y, yaw]
           tx, ty: target position
           phi_ref: target yaw
        """
        x = x0
        cost = 0.0
        for i in range(self.horizon):
            x = self.motion_model(x, u[i])
            # Compute yaw error relative to target yaw
            yaw_error = normalize_angle(x[2] - phi_ref)
            state_error = np.array([x[0] - tx, x[1] - ty, yaw_error])
            control_effort = np.array([u[i]])
            cost += state_error.T @ self.Q @ state_error + control_effort.T @ self.R @ control_effort
        return cost

    def motion_model(self, x, u):
        """Motion model for the robot.
           x: current state [x, y, yaw]
           u: control input (steering angle change) for one time step
           The model uses constant linear velocity.
        """
        x_next = np.zeros(3)
        x_next[0] = x[0] + self.linear_velocity * self.dt * math.cos(x[2])
        x_next[1] = x[1] + self.linear_velocity * self.dt * math.sin(x[2])
        x_next[2] = x[2] + u * self.dt
        return x_next

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("🛑 Robot Stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MPCPathTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
