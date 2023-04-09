#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum"),
                    "urdf",
                    "furuta_pendulum.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum"),
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
            FindPackageShare("furuta_pendulum"),
            "config",
            "simulation.yaml",
        ]
    )
    furuta_pendulum_simulation_node = Node(
        package="furuta_pendulum",
        executable="simulation_node",
        parameters=[simulation_params],
    )

    actions = [
        robot_state_pub_node,
        rviz_node,
        furuta_pendulum_simulation_node,
    ]

    return LaunchDescription(actions)
