#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_ocs2_ros"),
            "rviz",
            "furuta_pendulum.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output={"stdout": "log", "stderr": "log"},
    )

    actions = [
        rviz_node,
    ]

    return LaunchDescription(actions)
