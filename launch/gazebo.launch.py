#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
    )

    ign_config_file = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum"),
            "config",
            "furuta_pendulum_ignition.config",
        ]
    )
    ign_config_cfg = LaunchConfiguration("ign_config")
    declare_ign_config_arg = DeclareLaunchArgument(
        "ign_config", default_value=["-r empty.sdf --gui-config ", ign_config_file]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": ign_config_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "furuta_pendulum",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    forward_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_effort_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_forward_effort_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_effort_controller],
            )
        )
    )

    actions = [
        declare_ign_config_arg,
        SetParameter(name="use_sim_time", value=True),
        description_launch,
        gz_sim,
        gz_spawn_entity,
        ign_bridge,
        joint_state_broadcaster_spawner,
        delay_forward_effort_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(actions)
