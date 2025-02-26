import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PlotMapNode(Node):
    def __init__(self):
        super().__init__('plot_map_node')
        self.odom_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)
        self.path_x, self.path_y = self.load_path()
        self.ground_truth_x = []
        self.ground_truth_y = []
        
        self.fake_gps_subscriber = self.create_subscription(Odometry, '/fake_gps', self.fake_gps_callback, 10)
        self.fake_gps_x = []
        self.fake_gps_y = []

        self.ekf_subscriber = self.create_subscription(Odometry, '/ekf_yaw_rate', self.ekf_callback, 10)
        self.ekf_x = []
        self.ekf_y = []

        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.1, self.plot_data)
        self.save_timer = self.create_timer(1.0, self.save_odom_data)  # Save data every second

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
        return path_x, path_y

    def odom_callback(self, msg):
        self.ground_truth_x.append(msg.pose.pose.position.x)
        self.ground_truth_y.append(msg.pose.pose.position.y)

    def fake_gps_callback(self, msg):
        self.fake_gps_x.append(msg.pose.pose.position.x)
        self.fake_gps_y.append(msg.pose.pose.position.y)
    
    def ekf_callback(self, msg):
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)

    def plot_data(self):
        self.ax.clear()
        self.ax.plot(self.path_x, self.path_y, 'r-', label='Path')
        self.ax.plot(self.ground_truth_x, self.ground_truth_y, 'b-', label='Ground Truth')
        self.ax.plot(self.fake_gps_x, self.fake_gps_y, 'g-', label='Fake GPS')
        self.ax.plot(self.ekf_x, self.ekf_y, 'y-', label='EKF')
        self.ax.legend()
        plt.draw()
        plt.pause(0.01)

    def save_odom_data(self):
        path_file_save = '/home/kraiwich/FRA532_LAB1_ws/src/robot_plot/data/log/path.yaml'
        odom_data = [{'x': x, 'y': y} for x, y in zip(self.ground_truth_x, self.ground_truth_y)]
        with open(path_file_save, 'w') as file:
            yaml.safe_dump(odom_data, file)

def main(args=None):
    rclpy.init(args=args)
    node = PlotMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_odom_data()  # Save data one last time before shutting down
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()