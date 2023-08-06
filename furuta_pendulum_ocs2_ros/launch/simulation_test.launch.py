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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_params = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_ocs2_ros"),
            "config",
            "test_simulation.yaml",
        ]
    )
    simulation_node = Node(
        package="furuta_pendulum_ocs2_ros",
        executable="ocs2_test_simulation_node",
        parameters=[simulation_params],
    )

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

    actions = [simulation_node, rviz_node, description_launch]

    return LaunchDescription(actions)
