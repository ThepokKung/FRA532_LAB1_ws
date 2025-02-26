import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    pkg_name = 'robot_controller'
    node_name = ["ForwardKinematic-Double.py", "ForwardKinematic-Single.py", "ForwardKinematic-Yaw.py"]

    launch_description = LaunchDescription()

    for node in node_name:
        forward_kinematic_node = Node(
            package=pkg_name,
            executable=node,
            parameters=[{"use_sim_time": False}]
        )
        launch_description.add_action(forward_kinematic_node)

    return launch_description
