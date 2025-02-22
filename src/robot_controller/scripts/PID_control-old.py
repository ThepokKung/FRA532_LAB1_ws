#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import yaml,os

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Load path from YAML file
        """CSV PATH"""
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        # PID parameters
        self.kp = self.declare_parameter('kp', 1.0).value
        self.ki = self.declare_parameter('ki', 0.0).value
        self.kd = self.declare_parameter('kd', 0.0).value

        # Initialize errors
        self.prev_error = 0
        self.integral = 0

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/ground_truth/pose', self.pose_callback, 10)

        self.current_pose = None
        self.timer = self.create_timer(0.1, self.run)

        self.get_logger().info('PID Controller has started.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def compute_control(self, target_pose):
        if self.current_pose is None:
            return Twist()

        # Compute the error
        error_x = target_pose['x'] - self.current_pose.position.x
        error_y = target_pose['y'] - self.current_pose.position.y
        error = (error_x**2 + error_y**2)**0.5

        # PID calculations
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = control_signal
        cmd_vel.angular.z = 0  # For simplicity, we are not controlling angular velocity here

        return cmd_vel

    def run(self):
        for target_pose in self.path:
            while rclpy.ok():
                cmd_vel = self.compute_control(target_pose)
                self.cmd_vel_pub.publish(cmd_vel)
                rclpy.spin_once(self)

                # Check if the robot is close enough to the target pose
                if self.current_pose:
                    error_x = target_pose['x'] - self.current_pose.position.x
                    error_y = target_pose['y'] - self.current_pose.position.y
                    error = (error_x**2 + error_y**2)**0.5
                    if error < 0.1:  # Threshold to consider the target reached
                        break

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()