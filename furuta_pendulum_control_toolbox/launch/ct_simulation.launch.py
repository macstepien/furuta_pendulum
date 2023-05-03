#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
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

    simulation_params = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_control_toolbox"),
            "config",
            "ct_simulation.yaml",
        ]
    )
    furuta_pendulum_simulation_node = Node(
        package="furuta_pendulum_control_toolbox",
        executable="ct_demo_simulation_node",
        parameters=[simulation_params],
    )

    actions = [
        description_launch,
        rviz_node,
        furuta_pendulum_simulation_node,
    ]

    return LaunchDescription(actions)
