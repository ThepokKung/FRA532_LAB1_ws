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

    # ROBOT param
    spawn_x_val = '9.073500'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '1.57'

    robot_des_share = get_package_share_directory('robot_description')
    robot_con_share = get_package_share_directory('robot_controller')
    robot_sim_share = get_package_share_directory('robot_sim')
    rsp_file = os.path.join(robot_des_share, 'launch', 'rsp.launch.py')
    rviz_file = os.path.join(robot_con_share, 'rviz', 'displaysim3.rviz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                rsp_file
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    robot_sim_share,
                    "launch",
                    "world.launch.py"
                )
            ]
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "fake_limo",
            '-timeout', '120.0',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output="screen"
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[spawn_x_val, spawn_y_val, spawn_z_val, spawn_yaw_val, "0", "0", "world", "odom"],
        output="screen"
    )

    track_robot = Node(
        package="robot_sim",
        executable="track_robot.py",
        output="screen"
    )

    forward_kinematics = Node(
        package="robot_controller",
        executable="ForwardKinematic-All.py",
        output="screen"
    )

    invert_kinematics = Node(
        package="robot_controller",
        executable="InverKinematic-nscc.py",
        output="screen"
    )

    fake_gps = Node(
        package="robot_fake",
        executable="fake_gps.py",
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )
    
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    steering_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file
        ],
        output = "screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velocity_controller_spawner,
                on_exit=[steering_position_controller_spawner],
            )
        )
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(rsp)
    launch_description.add_action(static_transform_publisher)
    launch_description.add_action(forward_kinematics)
    launch_description.add_action(invert_kinematics)
    launch_description.add_action(fake_gps)
    launch_description.add_action(track_robot)
    return launch_description
