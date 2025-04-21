#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_names = ['EKF-Yaw', 'EKF-Single', 'EKF-Double']
    
    launch_nodes = []
    
    for name in node_names:
        launch_nodes.append(
            Node(
                package='robot_controller',
                executable=name+'.py',
                name=f'node_{name.replace("-", "_")}',
                output='screen',
            )
        )

    return LaunchDescription(launch_nodes)