import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf')
    # urdf_file = os.path.join(pkg_share, 'robot', 'visual', 'robot.urdf.xacro')
    rviz_launch_file = os.path.join(
        pkg_share, 'launch', 'rviz-display.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch_file])
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'fake-limo', '-file', urdf_file],
            output='screen'
        ),
    ])
