from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'rviz-display.launch.py'
            ])
        ])
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': PathJoinSubstitution([
            FindPackageShare('robot_sim'),
            'world',
            'basic.world'
        ])}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'limo', '-file', PathJoinSubstitution([
            FindPackageShare('robot_description'),
            'robot',
            'visual',
            'robot.urdf'
        ])],
        output='screen'
    )

    return LaunchDescription([
        robot_description_launch,
        gazebo_launch,
        spawn_entity
    ])