#!/usr/bin/env python3
import os , xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    rviz_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rsp_file = os.path.join(pkg_share, 'launch', 'rsp.launch.py')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                rsp_file
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(rsp)
    # launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    return launch_description

