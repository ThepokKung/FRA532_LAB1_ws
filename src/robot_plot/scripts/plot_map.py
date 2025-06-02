#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import math

class PlotMapNode(Node):
    def __init__(self):
        super().__init__('plot_map_node')
        
        # Track initial poses for offset correction
        self.initial_poses = {
            'ground_truth': None,
            'yaw_rate': None,
            'single_track': None,
            'double_track': None
        }
        
        # Subscribers
        self.odom_gt_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_gt_callback, 10)
        self.odom_yaw_subscriber = self.create_subscription(Odometry, '/odom_yaw_rate', self.odom_yaw_callback, 10)
        self.odom_single_subscriber = self.create_subscription(Odometry, '/odom_single_track', self.odom_single_callback, 10)
        self.odom_double_subscriber = self.create_subscription(Odometry, '/odom_double_track', self.odom_double_callback, 10)

        self.path_x, self.path_y, self.path_yaw = self.load_path()
        
        # Ground truth data
        self.odom_gt_x = []
        self.odom_gt_y = []
        self.odom_gt_yaw = []
        self.odom_gt_ang_z = []

        # YawRate data
        self.odom_yaw_x = []
        self.odom_yaw_y = []
        self.odom_yaw_yaw = []
        self.odom_yaw_ang_z = []

        # Single track data
        self.odom_single_x = []
        self.odom_single_y = []
        self.odom_single_yaw = []
        self.odom_single_ang_z = []

        # Double track data
        self.odom_double_x = []
        self.odom_double_y = []
        self.odom_double_yaw = []
        self.odom_double_ang_z = []

        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.1, self.plot_data)
        self.save_timer = self.create_timer(0.1, self.save_odom_data)

    def load_path(self):
        path_file = os.path.join(
            get_package_share_directory('robot_controller'),
            'config',
            'path.yaml'
        )
        with open(path_file, 'r') as file:
            path_data = yaml.safe_load(file)
        path_x = [pose['x'] for pose in path_data]
        path_y = [pose['y'] for pose in path_data]
        path_yaw = [pose['yaw'] for pose in path_data]
        return path_x, path_y, path_yaw

    def process_odom(self, msg, key):
        """Process any odometry message with simple position offset"""
        # Extract position and orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        ang_z = msg.twist.twist.angular.z
        
        # Store the initial pose if not already set
        if self.initial_poses[key] is None:
            self.initial_poses[key] = {'x': x, 'y': y, 'yaw': yaw}
            self.get_logger().info(f"Initial {key} pose set: x={x}, y={y}, yaw={yaw}")
            
        # If we have ground truth initial pose, we can apply a simple position offset
        if self.initial_poses['ground_truth'] is not None and self.initial_poses[key] is not None:
            # Calculate initial position offsets
            x_offset = self.initial_poses[key]['x'] - self.initial_poses['ground_truth']['x']
            y_offset = self.initial_poses[key]['y'] - self.initial_poses['ground_truth']['y']
            
            # Apply simple position offset (no rotation)
            x_corrected = x - x_offset
            y_corrected = y - y_offset
            
            return x_corrected, y_corrected, yaw, ang_z
        else:
            # Return raw data if we can't apply correction yet
            return x, y, yaw, ang_z

    def odom_gt_callback(self, msg):
        x, y, yaw, ang_z = self.process_odom(msg, 'ground_truth')
        self.odom_gt_x.append(x)
        self.odom_gt_y.append(y)
        self.odom_gt_yaw.append(yaw)
        self.odom_gt_ang_z.append(ang_z)

    def odom_yaw_callback(self, msg):
        x, y, yaw, ang_z = self.process_odom(msg, 'yaw_rate')
        self.odom_yaw_x.append(x)
        self.odom_yaw_y.append(y)
        self.odom_yaw_yaw.append(yaw)
        self.odom_yaw_ang_z.append(ang_z)

    def odom_single_callback(self, msg):
        x, y, yaw, ang_z = self.process_odom(msg, 'single_track')
        self.odom_single_x.append(x)
        self.odom_single_y.append(y)
        self.odom_single_yaw.append(yaw)
        self.odom_single_ang_z.append(ang_z)
    
    def odom_double_callback(self, msg):
        x, y, yaw, ang_z = self.process_odom(msg, 'double_track')
        self.odom_double_x.append(x)
        self.odom_double_y.append(y)
        self.odom_double_yaw.append(yaw)
        self.odom_double_ang_z.append(ang_z)
    
    def plot_data(self):
        # Add this to the beginning of the plot_data method
        if len(self.odom_gt_x) > 0 and len(self.odom_yaw_x) > 0:
            gt_pos = (self.odom_gt_x[-1], self.odom_gt_y[-1], self.odom_gt_yaw[-1])
            yaw_pos = (self.odom_yaw_x[-1], self.odom_yaw_y[-1], self.odom_yaw_yaw[-1])
            self.get_logger().debug(f"Latest GT: ({gt_pos[0]:.2f}, {gt_pos[1]:.2f}, {gt_pos[2]:.2f}) | " +
                                    f"Yaw: ({yaw_pos[0]:.2f}, {yaw_pos[1]:.2f}, {yaw_pos[2]:.2f})")

        # If we don't have multiple axes yet, create them
        if not hasattr(self, 'axes'):
            plt.close(self.fig)
            self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 15))
            plt.tight_layout(pad=3.0)
            self.get_logger().info("Created plotting axes")
        
        for ax in self.axes:
            ax.clear()

        # Define common plotting parameters
        labels = ['Ground Truth', 'Yaw Rate', 'Single Track', 'Double Track']
        colors = ['blue', 'red', 'green', 'purple']
        styles = ['-', '--', '--', '--']
        
        # Get data sets
        x_data = [self.odom_gt_x, self.odom_yaw_x, self.odom_single_x, self.odom_double_x]
        y_data = [self.odom_gt_y, self.odom_yaw_y, self.odom_single_y, self.odom_double_y]
        yaw_data = [self.odom_gt_yaw, self.odom_yaw_yaw, self.odom_single_yaw, self.odom_double_yaw]
        ang_z_data = [self.odom_gt_ang_z, self.odom_yaw_ang_z, self.odom_single_ang_z, self.odom_double_ang_z]

        # Plot path on the first subplot
        self.axes[0].plot(self.path_x, self.path_y, 'k-', label='Path')
        
        # XY Position
        for i, (x, y, label, color, style) in enumerate(zip(x_data, y_data, labels, colors, styles)):
            if len(x) > 0 and len(y) > 0:
                self.axes[0].plot(x, y, linestyle=style, color=color, label=label)
        self.axes[0].set_title('XY Position')
        self.axes[0].set_xlabel('X Coordinate')
        self.axes[0].set_ylabel('Y Coordinate')
        self.axes[0].legend()
        self.axes[0].grid(True)

        # Orientation (Yaw)
        for i, (yaw, label, color, style) in enumerate(zip(yaw_data, labels, colors, styles)):
            if len(yaw) > 0:
                self.axes[1].plot(yaw, linestyle=style, color=color, label=label)
        self.axes[1].set_title('Orientation (Yaw)')
        self.axes[1].set_xlabel('Time Step')
        self.axes[1].set_ylabel('Yaw (rad)')
        self.axes[1].legend()
        self.axes[1].grid(True)

        # Angular Velocity Z
        for i, (ang_z, label, color, style) in enumerate(zip(ang_z_data, labels, colors, styles)):
            if len(ang_z) > 0:
                self.axes[2].plot(ang_z, linestyle=style, color=color, label=label)
        self.axes[2].set_title('Angular Velocity Z')
        self.axes[2].set_xlabel('Time Step')
        self.axes[2].set_ylabel('Angular Velocity Z (rad/s)')
        self.axes[2].legend()
        self.axes[2].grid(True)

        plt.draw()
        plt.pause(0.01)

    def save_odom_data(self):
        # Create the directory if it doesn't exist
        save_dir = '/home/kraiwich/FRA532_LAB1_ws/src/robot_plot/data/log'
        os.makedirs(save_dir, exist_ok=True)
        
        path_file_save = os.path.join(save_dir, 'odom_data.yaml')
        
        # Collect all data into a structured dictionary
        odom_data = {
            'ground_truth': [
                {'x': x, 'y': y, 'yaw': yaw, 'angular_z': ang_z} 
                for x, y, yaw, ang_z in zip(self.odom_gt_x, self.odom_gt_y, self.odom_gt_yaw, self.odom_gt_ang_z)
            ],
            'yaw_rate': [
                {'x': x, 'y': y, 'yaw': yaw, 'angular_z': ang_z} 
                for x, y, yaw, ang_z in zip(self.odom_yaw_x, self.odom_yaw_y, self.odom_yaw_yaw, self.odom_yaw_ang_z)
            ],
            'single_track': [
                {'x': x, 'y': y, 'yaw': yaw, 'angular_z': ang_z} 
                for x, y, yaw, ang_z in zip(self.odom_single_x, self.odom_single_y, self.odom_single_yaw, self.odom_single_ang_z)
            ],
            'double_track': [
                {'x': x, 'y': y, 'yaw': yaw, 'angular_z': ang_z} 
                for x, y, yaw, ang_z in zip(self.odom_double_x, self.odom_double_y, self.odom_double_yaw, self.odom_double_ang_z)
            ],
            'initial_poses': self.initial_poses
        }
        
        with open(path_file_save, 'w') as file:
            yaml.safe_dump(odom_data, file)
        
        self.get_logger().info(f"Saved odometry data to {path_file_save}")
        
        # Also save the plot as an image
        plot_dir = '/home/kraiwich/FRA532_LAB1_ws/src/robot_plot/data/plots'
        os.makedirs(plot_dir, exist_ok=True)
        plot_file = os.path.join(plot_dir, 'odometry_comparison.png')
        self.fig.savefig(plot_file, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Saved plot to {plot_file}")

def main(args=None):
    rclpy.init(args=args)
    node = PlotMapNode()
    try:
        plt.ion()  # Enable interactive mode
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_odom_data()  # Save data one last time before shutting down
        node.destroy_node()
        plt.ioff()
        rclpy.shutdown()

if __name__ == '__main__':
    main()