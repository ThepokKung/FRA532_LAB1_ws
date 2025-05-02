#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_sim = get_package_share_directory('robot_sim')

    # Launch arguments
    steering_mode_arg = DeclareLaunchArgument(
        'steering_mode',
        default_value='nscc',
        description='Choose "basic" or "nscc" for steering mode'
    )

    # Simulation launch
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_sim, 'launch', 'sim.launch.py')
        )
    )

    # Steering model nodes (conditionally started)
    steering_model_basic = Node(
        package='robot_controller',
        executable='InverKinematic-basic.py',
        name='steering_model_node_basic',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('steering_mode'), "' == 'basic'"]))
    )

    steering_model_nscc = Node(
        package='robot_controller',
        executable='InverKinematic-nscc.py',
        name='steering_model_node_nscc',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('steering_mode'), "' == 'nscc'"]))
    )

    gps_node = Node(
        package='robot_fake',
        executable='fake_gps.py',
        name='gps_node'
    )
    
    path_pub_node = Node(
        package='robot_sim',
        executable='path_pub.py',
        name='path_pub_node'
    )

    delayed_path_pub = TimerAction(
        period=2.0,          # wait 2 seconds
        actions=[path_pub_node]
    )

    path_track_node = Node(
        package='robot_sim',
        executable='path_track_robot.py',
        name='path_track_node'
    )

    return LaunchDescription([
        steering_mode_arg,
        sim,
        steering_model_basic,
        steering_model_nscc,
        gps_node,
        # path_pub_node,
        delayed_path_pub,
        path_track_node
    ])
