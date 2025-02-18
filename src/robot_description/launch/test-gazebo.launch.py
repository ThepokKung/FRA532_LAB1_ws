import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    # urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf')
    urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf.xacro')
    rviz_launch_file = os.path.join(pkg_share, 'launch', 'rviz-display.launch.py')
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    controller_config = os.path.join(pkg_share, 'config', 'fake_limo_controllers.yaml')

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file])
        ),

        # Launch RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch_file])
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'fake-limo', '-file', urdf_file],
            output='screen'
        ),

        # Start ros2_control_node (controller manager)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controller_config],
            output="screen"
        ),

        # Delay to ensure controller manager is ready
        TimerAction(
            period=5.0,  # Wait 5 seconds before spawning controllers
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen"
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["diff_drive_controller"],
                    output="screen"
                )
            ]
        ),
    ])
