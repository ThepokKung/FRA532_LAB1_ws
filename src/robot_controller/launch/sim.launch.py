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
    robot_des_share = get_package_share_directory('robot_description')
    robot_con_share = get_package_share_directory('robot_controller')
    rsp_file = os.path.join(robot_des_share, 'launch', 'rst.launch.py')
    rviz_file = os.path.join(robot_con_share, 'rviz', 'displaysim.rviz')

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', rviz_file],
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
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
        ],
        output = "screen"
    )
    
    controller = Node(
    	package="my_controller",
    	executable="diff_drive.py"
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

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    # launch_description.add_action(controller)
    launch_description.add_action(rsp)
    return launch_description
