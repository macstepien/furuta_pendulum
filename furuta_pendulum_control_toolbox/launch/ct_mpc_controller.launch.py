#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_control_toolbox"),
            "config",
            "mpc_params.yaml",
        ]
    )

    furuta_pendulum_controller_node = Node(
        package="furuta_pendulum_control_toolbox",
        executable="ct_mpc_controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[controller_params],
    )

    actions = [
        furuta_pendulum_controller_node,
    ]

    return LaunchDescription(actions)
