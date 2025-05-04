#!/usr/bin/env python3
import os
import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PathTrackingStanley(Node):
    def __init__(self):
        super().__init__('path_tracking_stanley')

        # YAML file path
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config', 'path.yaml'
        )

        # Load the path from the YAML file
        if os.path.exists(path_file):
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
            self.path = data.get('path', data) if isinstance(data, dict) else data
            self.get_logger().info(
                f"✅ Loaded path with {len(self.path)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

        # Parameters
        self.declare_parameter('use_ekf', False)
        self.declare_parameter('k', 1.0)
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('goal_threshold', 0.1)

        self.use_ekf = self.get_parameter('use_ekf').value
        self.k = self.get_parameter('k').value
        self.v_const = self.get_parameter('linear_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.goal_threshold = self.get_parameter('goal_threshold').value

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Path tracking indices
        self.path_index = 0
        self._init_targets()

        # Subscriber
        odom_topic = '/ekf/odom' if self.use_ekf else '/ground_truth/pose'
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)

        # Initialization logs
        self.get_logger().info("🚀 Stanley Controller Initialized")
        self.get_logger().info(f"   K: {self.k:.2f}")
        self.get_logger().info(f"   Linear Velocity: {self.v_const:.2f} m/s")


    def _init_targets(self):
        # Initialize previous and current segment points
        if len(self.path) < 2:
            self.previous = self.path[0] if self.path else {'x': 0, 'y': 0}
            self.current = self.previous
        else:
            self.previous = self.path[0]
            self.current = self.path[1]

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

    def publish_cmd(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        # No path
        if not self.path:
            self.publish_cmd(0.0, 0.0)
            return

        # Compute distance to current target
        dx = self.current['x'] - self.current_x
        dy = self.current['y'] - self.current_y
        dist = math.hypot(dx, dy)

        # If at final target and within threshold => stop
        if self.path_index >= len(self.path) - 1 and dist < self.goal_threshold:
            self.get_logger().info('🏁 Reached final goal. Stopping.')
            self.publish_cmd(0.0, 0.0)
            return

        # Advance to next segment if close to current target
        if dist < self.goal_threshold and self.path_index < len(self.path) - 1:
            self.path_index += 1
            self.previous = self.current
            next_idx = min(self.path_index + 1, len(self.path) - 1)
            self.current = self.path[next_idx]
            return

        # Compute cross-track error to line segment
        x1, y1 = self.previous['x'], self.previous['y']
        x2, y2 = self.current['x'], self.current['y']
        seg_dx = x2 - x1
        seg_dy = y2 - y1
        if seg_dx == 0 and seg_dy == 0:
            cross_err = 0.0
        else:
            cross_err = ((self.current_x - x1) * seg_dy -
                         (self.current_y - y1) * seg_dx) / math.hypot(seg_dx, seg_dy)

        # Heading error
        path_yaw = math.atan2(seg_dy, seg_dx)
        heading_err = normalize_angle(path_yaw - self.current_yaw)

        # Stanley control law
        delta = heading_err + math.atan2(self.k * cross_err, self.v_const)
        delta = normalize_angle(delta)
        omega = (self.v_const / self.wheelbase) * math.tan(delta)

        # Publish
        self.publish_cmd(self.v_const, omega)

        # Log information
        self.get_logger().info(f'🎯 Target waypoint: {x2:.3f}, {y2:.3f}')
        self.get_logger().info(f'   Distance Error: {dx:.3f}, Yaw Error: {dy:.3f}')
        self.get_logger().info(f'   Control Linear: {self.v_const:.3f}, Control Angular: {omega:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingStanley()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
