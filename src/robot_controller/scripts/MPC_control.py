#!/usr/bin/python3
"""
mpc_path_tracking.py

Model Predictive Control (MPC) Controller for an Ackermann Steering Mobile Robot,
where the control vector is u = [a, delta] with:
  - a: acceleration (m/s²)
  - delta: steering angle (rad)

State vector: [x, y, v, phi]^T, where:
  - x, y: position (m)
  - v: speed (m/s)
  - phi: orientation (rad)

Dynamics (linearized about the current state, with small steering angle assumption):
  x[k+1]   = x[k] + dt * v[k] * cos(phi0)
  y[k+1]   = y[k] + dt * v[k] * sin(phi0)
  v[k+1]   = v[k] + dt * a[k]
  phi[k+1] = phi[k] + dt * (v0 * delta[k] / L)

Here, phi0 and v0 are the current orientation and speed (treated as constants over the horizon).
The MPC minimizes the quadratic cost of tracking error and control effort.
In this version, the computed steering angle delta is sent directly as cmd_vel.angular.z,
because the Inverse Kinematics node expects a steering angle.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
import math
import os
import numpy as np
import cvxpy as cp
from ament_index_python.packages import get_package_share_directory

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class MPCPathTracking(Node):
    def __init__(self):
        super().__init__('mpc_path_tracking')
        
        # Load path from YAML file (expected format: list of waypoints with keys 'x' and 'y')
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        try:
            with open(path_file, 'r') as f:
                self.path = yaml.safe_load(f)
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load path: {e}")
            self.path = []
        
        # MPC parameters
        self.declare_parameter('prediction_horizon', 10)  # Prediction horizon (number of steps)
        self.declare_parameter('dt', 0.1)                 # Time step (s)
        self.N = self.get_parameter('prediction_horizon').value
        self.dt = self.get_parameter('dt').value
        
        # Vehicle parameter: wheelbase L (m)
        self.declare_parameter('wheelbase', 0.20)
        self.L = self.get_parameter('wheelbase').value
        
        # Cost matrices (tunable)
        self.Q = np.diag([10.0, 10.0, 1.0, 1.0])    # State tracking cost
        self.R_mat = np.diag([0.1, 0.1])              # Control effort cost
        self.Qf = self.Q.copy()                       # Terminal state cost
        
        # Input constraints
        self.a_max = 1.0          # Maximum acceleration (m/s^2)
        self.delta_max = 0.5236   # Maximum steering angle (rad) ≈ 30°
        
        # Reference speed
        self.v_ref = 1.0          # m/s
        
        # Waypoint threshold for switching target
        self.declare_parameter('waypoint_threshold', 0.5)
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.current_target_index = 0
        
        # Initial state (will be updated from odometry callback); state: [x, y, v, phi]
        self.x0 = None
        
        # Subscriber: current pose (Odometry) from /ground_truth/pose
        self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        # Publisher for cmd_vel command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.prev_time = None
        
        self.get_logger().info("🚀 MPC Path Tracking Node Initialized")
    
    def odom_callback(self, msg: Odometry):
        # Extract current state from Odometry
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        phi = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        v = msg.twist.twist.linear.x
        self.x0 = np.array([cx, cy, v, phi])
        
        # Check if there are remaining waypoints
        if self.current_target_index >= len(self.path):
            self.get_logger().info("🏁 No more waypoints. Stopping robot.")
            self.publish_stop()
            return
        
        # Generate reference trajectory for the next N+1 steps
        ref_traj = self.generate_reference_trajectory()
        
        # Solve MPC problem
        u_opt = self.solve_mpc(self.x0, ref_traj)
        if u_opt is None:
            self.get_logger().warn("MPC did not solve optimally. Stopping robot.")
            self.publish_stop()
            return
        
        # Use only the first control input: u0 = [a, delta]
        a_cmd = u_opt[0, 0]
        delta_cmd = u_opt[1, 0]
        
        # Convert steering angle δ to angular velocity ω
        angular_velocity_cmd = delta_cmd * self.v_ref / self.L
        
        # Log target position
        target_wp = self.path[self.current_target_index]
        tx = target_wp['x']
        ty = target_wp['y']
        self.get_logger().info(f"🎯 Target Position: x={tx:.2f}, y={ty:.2f}")
        
        # Create and publish cmd_vel message
        cmd = Twist()
        cmd.linear.x = self.v_ref
        cmd.angular.z = angular_velocity_cmd
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"🚗 MPC Command: a = {a_cmd:.2f}, δ = {delta_cmd:.2f} (converted to angular velocity ω = {angular_velocity_cmd:.2f})")
    
    def generate_reference_trajectory(self):
        """
        Generate reference trajectory for the next N+1 steps based on the path.
        Each reference state: [x_ref, y_ref, v_ref, phi_ref]
        If there are fewer than N+1 waypoints, repeat the last waypoint.
        Returns a numpy array of shape (4, N+1).
        """
        traj = []
        idx = self.current_target_index
        for i in range(self.N + 1):
            if idx >= len(self.path):
                wp = self.path[-1]
            else:
                wp = self.path[idx]
            x_ref = wp['x']
            y_ref = wp['y']
            # For phi_ref, compute desired heading from current waypoint to next if available.
            if idx + 1 < len(self.path):
                next_wp = self.path[idx+1]
                phi_ref = math.atan2(next_wp['y'] - wp['y'], next_wp['x'] - wp['x'])
            else:
                phi_ref = 0.0
            traj.append([x_ref, y_ref, self.v_ref, phi_ref])
            idx += 1
        return np.array(traj).T  # Shape: (4, N+1)
    
    def solve_mpc(self, x0, ref_traj):
        """
        Solve the MPC optimization problem using CVXPY with linearized dynamics.
        
        State vector: [x, y, v, phi]
        Control vector: [a, delta]
        
        Linearized Dynamics (using current phi0 and v0):
            x[k+1]   = x[k] + dt * v[k] * cos(phi0)
            y[k+1]   = y[k] + dt * v[k] * sin(phi0)
            v[k+1]   = v[k] + dt * a[k]
            phi[k+1] = phi[k] + dt * (v0 * delta[k] / L)
        
        Here, phi0 and v0 are taken from the current state x0.
        """
        n_state = 4
        n_control = 2
        N = self.N
        
        phi0 = x0[3]
        v0 = x0[2]
        cos_phi0 = math.cos(phi0)
        sin_phi0 = math.sin(phi0)
        
        # Define optimization variables
        x = cp.Variable((n_state, N+1))
        u = cp.Variable((n_control, N))
        
        cost = 0
        constraints = []
        # Initial condition
        constraints += [x[:, 0] == x0]
        
        for k in range(N):
            # Linearized dynamics constraints
            constraints += [
                x[0, k+1] == x[0, k] + self.dt * x[2, k] * cos_phi0,
                x[1, k+1] == x[1, k] + self.dt * x[2, k] * sin_phi0,
                x[2, k+1] == x[2, k] + self.dt * u[0, k],
                x[3, k+1] == x[3, k] + self.dt * (v0 * u[1, k] / self.L)
            ]
            # Input constraints
            constraints += [
                cp.abs(u[0, k]) <= self.a_max,
                cp.abs(u[1, k]) <= self.delta_max
            ]
            # Stage cost: tracking error (state error)
            cost += cp.sum_squares(x[:, k+1] - ref_traj[:, k+1])
            # Control effort cost
            cost += cp.sum_squares(u[:, k])
        
        # Terminal cost
        cost += cp.sum_squares(x[:, N] - ref_traj[:, N])
        
        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True)
        except Exception as e:
            self.get_logger().error(f"MPC solver error: {e}")
            return None
        
        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            return u.value
        else:
            self.get_logger().warn("MPC did not solve optimally.")
            return None
    
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
