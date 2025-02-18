#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    # urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf')
    urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # joint_state_publisher = Node(
    #      package='joint_state_publisher',
    #      executable = 'joint_state_publisher',
    #      name = 'joint_state_publisher',
    #      output = 'screen'
    # )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    return launch_description
