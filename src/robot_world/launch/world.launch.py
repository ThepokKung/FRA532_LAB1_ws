#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get the package share directory for ros_gz_sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Get the package share directory for your custom package
    limo_world_pkg_path = get_package_share_directory(
        'robot_world')  # Replace with your package name

    # Path to the Gazebo launch file
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Path to the World file
    world_path = PathJoinSubstitution(
        [limo_world_pkg_path, 'world', 'basic.world'])

    return LaunchDescription([
        # Declare the world_path launch configuration
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([limo_world_pkg_path, 'models'])
        ),
        # Declare the world_path as a launch argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                #    'gz_args': [world_path, ' -v 4'],  # Combine world path with verbose flag
                # Combine world path with verbose flag
                'gz_args': [world_path,],
                'on_exit_shutdown': 'True'
            }.items(),
        )
    ])
