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
            FindPackageShare("furuta_pendulum_de"),
            "config",
            "lqr_with_swingup_controller.yaml",
        ]
    )

    pendulum_params = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_description"),
            "config",
            "pendulum_parameters.yaml",
        ]
    )

    furuta_pendulum_controller_node = Node(
        package="furuta_pendulum_de",
        executable="lqr_with_swingup_controller_node",
        parameters=[controller_params, pendulum_params],
    )

    actions = [
        furuta_pendulum_controller_node,
    ]

    return LaunchDescription(actions)