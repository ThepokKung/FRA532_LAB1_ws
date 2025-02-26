#!/usr/bin/python3
"""
path_tracking_pid_continuous_odom.py

PID Path Tracking Controller for Ackermann Steering Mobile Robot

- Load path from YAML file (from package 'robot_controller/config/path.yaml')
- Subscribe to ground truth odometry from /ground_truth/odom
- Continuously follow the path from the first waypoint to the final waypoint.
- Use PID on heading error (difference between desired heading toward the target waypoint and current heading)
- When the robot is close enough to the target waypoint (within a threshold), move on to the next waypoint.
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

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class PIDPathTracking(Node):
    def __init__(self):
        super().__init__('pid_path_tracking')

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

        # PID gains (tunable)
        self.declare_parameter('Kp', 20.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # Lookahead threshold: distance threshold to consider a waypoint reached
        self.declare_parameter('lookahead_threshold', 0.5)
        self.lookahead_threshold = self.get_parameter('lookahead_threshold').value

        # Constant linear velocity (m/s)
        self.declare_parameter('linear_velocity', 20.0)
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # PID state variables
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Current target index in the path
        self.current_target_index = 0

        self.create_subscription(Odometry, '/ekf_yaw_rate', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🚀 PID Path Tracking Node Initialized")
        self.get_logger().info(f"🔹 Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
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
            desired_heading = math.atan2(ty - cy, tx - cx)
            heading_error = normalize_angle(desired_heading - current_yaw)

        # Compute time step
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            dt = 0.01
        else:
            dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # PID control for steering (angular velocity command)
        self.integral_error += heading_error * dt
        derivative = (heading_error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = heading_error
        angular_velocity_cmd = self.Kp * heading_error + self.Ki * self.integral_error + self.Kd * derivative

        self.get_logger().info(f"🔄 PID: heading_err={math.degrees(heading_error):.2f}°, integral={self.integral_error:.3f}, derivative={math.degrees(derivative):.2f}°, angular_velocity_cmd={math.degrees(angular_velocity_cmd):.2f}°/s")

        # Create and publish cmd_vel message
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity_cmd
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"🚗 Publishing cmd_vel: linear.x={cmd.linear.x:.2f} m/s, angular.z={math.degrees(cmd.angular.z):.2f}°/s")

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("🛑 Robot Stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = PIDPathTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
