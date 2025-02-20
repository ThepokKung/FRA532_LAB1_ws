#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = "robot_controller"
    pkg_share = get_package_share_directory('robot_controller')

    forward_kinematic_modle = ['ForwardKinematic-Double.py','ForwardKinematic-Single.py','ForwardKinematic-Yaw.py'] 

    launch_description = LaunchDescription()

    nodes = []
    for model in forward_kinematic_modle:
        node = Node(
            package=pkg_name,
            executable=model,
            output='screen'
        )
        launch_description.add_action(node)


    return launch_description
