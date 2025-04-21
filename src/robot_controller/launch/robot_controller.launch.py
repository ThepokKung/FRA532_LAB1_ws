#!/usr/bin/env python3
import os


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='pure_pursuit',
        description='Control mode'
    )

    PID_controller = Node(
        package='robot_controller',
        executable='PID_control.py',
        name='PID_controller_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'pid'"]))
    )

    PurePursuit_controller = Node(
        package='robot_controller',
        executable='PurePursuit_control.py',
        name='PurePursuit_controller_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'pure_pursuit'"]))
    )

    Stanley_controller = Node(
        package='robot_controller',
        executable='Stanley_control.py',
        name='Stanley_controller_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'stanley'"]))
    )

    return LaunchDescription([
        control_mode_arg,
        PID_controller,
        PurePursuit_controller,
        Stanley_controller
    ])
