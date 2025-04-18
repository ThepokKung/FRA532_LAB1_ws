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
    pkg_name = "robot_description"
    pkg_share = get_package_share_directory('robot_description')
    # urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf')
    # urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf.xacro')
    # xacro_file = os.path.join(pkg_share, 'description', 'robot-main.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
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
                # os.path.join(
                #     get_package_share_directory(pkg_name),
                #     "launch",
                #     "rsp.launch.py"  # Ensure this file exists in the specified directory
                # )
                rsp_file
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )
    
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

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
    launch_description.add_action(rsp)
    # launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    return launch_description

