#!/usr/bin/python3
"""
pure_pursuit_controller_direct_angle.py

Pure Pursuit Controller for Ackermann Steering Mobile Robot.
This version outputs the desired front wheel steering angle (δ) directly
via cmd_vel.angular.z, assuming the downstream Inverse Kinematics node
interprets angular.z as the steering angle.
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
    return math.atan2(math.sin(angle), math.cos(angle))

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Load path from YAML file
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        try:
            with open(path_file, 'r') as f:
                self.path = yaml.safe_load(f)
            self.get_logger().info(f"✅ Loaded path with {len(self.path)} waypoints")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load path: {e}")
            self.path = []
        
        # Pure Pursuit parameters
        self.declare_parameter('K_dd', 1.0)         # Lookahead gain
        self.declare_parameter('l_min', 0.5)          # Minimum lookahead distance (m)
        self.declare_parameter('l_max', 3.0)          # Maximum lookahead distance (m)
        self.K_dd = self.get_parameter('K_dd').value
        self.l_min = self.get_parameter('l_min').value
        self.l_max = self.get_parameter('l_max').value
        
        # Vehicle parameter: wheelbase L (m)
        self.declare_parameter('wheelbase', 0.20)
        self.L = self.get_parameter('wheelbase').value

        # Control parameters
        self.declare_parameter('linear_velocity', 20)  # m/s
        self.linear_velocity = self.get_parameter('linear_velocity').value
        
        # Threshold for waypoint reaching
        self.declare_parameter('waypoint_threshold', 1.0)  # m
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        
        # Current target index
        self.current_target_index = 0
        
        # Subscriber for current pose (Odometry) from ground truth or state estimator
        self.create_subscription(Odometry, '/ekf_yaw_rate', self.odom_callback, 10)
        # Publisher for cmd_vel command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.prev_time = None
        
        self.get_logger().info("🚀 Pure Pursuit Controller Node Initialized (Direct Angle Mode)")
        self.get_logger().info(f"🔹 Lookahead: K_dd={self.K_dd}, l_min={self.l_min}, l_max={self.l_max}")
        self.get_logger().info(f"🔹 Wheelbase: {self.L} m, Linear Velocity: {self.linear_velocity} m/s")
    
    def odom_callback(self, msg: Odometry):
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        v = msg.twist.twist.linear.x
        
        # Dynamic lookahead distance: l_d = clip(K_dd * v, l_min, l_max)
        l_d = max(min(self.K_dd * v, self.l_max), self.l_min)
        
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
            self.get_logger().info(f"✅ Reached waypoint {self.current_target_index} (error={distance_error:.2f} m)")
            self.current_target_index += 1
            if self.current_target_index >= len(self.path):
                self.get_logger().info("🏁 Final waypoint reached. Stopping.")
                self.publish_stop()
                return
            target_wp = self.path[self.current_target_index]
            tx = target_wp['x']
            ty = target_wp['y']
        
        desired_heading = math.atan2(ty - cy, tx - cx)
        alpha = normalize_angle(desired_heading - current_yaw)
        
        # Compute desired steering angle δ using Pure Pursuit formula:
        # δ = arctan((2 * L * sin(α)) / l_d)
        delta = math.atan((2 * self.L * math.sin(alpha)) / l_d)
        
        # Convert steering angle δ to angular velocity ω
        angular_velocity_cmd = delta * self.linear_velocity / self.L
        
        # Log debug info
        self.get_logger().info(f"📍 Pose: x={cx:.2f}, y={cy:.2f}, yaw={math.degrees(current_yaw):.1f}°")
        self.get_logger().info(f"🎯 Target[{self.current_target_index}]: x={tx:.2f}, y={ty:.2f}, Dist Err: {distance_error:.2f} m")
        self.get_logger().info(f"   Desired Heading: {math.degrees(desired_heading):.1f}°, α: {math.degrees(alpha):.1f}°")
        self.get_logger().info(f"   Lookahead: {l_d:.2f} m, Steering δ: {math.degrees(delta):.1f}°, Angular Velocity ω: {math.degrees(angular_velocity_cmd):.1f}°/s")
        
        # Create and publish cmd_vel message
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity_cmd
        self.cmd_pub.publish(cmd)
    
    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("🛑 Robot Stopped.")
        
def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
