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

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum_ocs2_ros"),
                    "launch",
                    "visualize.launch.py",
                ]
            )
        ),
    )

    mpc_mrt_node = Node(
        package="furuta_pendulum_ocs2_ros",
        executable="furuta_pendulum_mpc_mrt",
    )

    actions = [
        description_launch,
        visualization_launch,
        TimerAction(period=4.0, actions=[mpc_mrt_node]),
    ]

    return LaunchDescription(actions)
