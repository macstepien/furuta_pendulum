#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_de"),
            "rviz",
            "furuta_pendulum.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    furuta_pendulum_simulation_node = Node(
        package="furuta_pendulum_control_toolbox",
        executable="ct_demo_simulation_mpc_node",
    )

    actions = [
        description_launch,
        rviz_node,
        TimerAction(period=4.0, actions=[furuta_pendulum_simulation_node]),
    ]

    return LaunchDescription(actions)
