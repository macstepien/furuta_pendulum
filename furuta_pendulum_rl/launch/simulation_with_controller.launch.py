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
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
    )
    furuta_pendulum_rl_controller_node = Node(
        package="furuta_pendulum_rl",
        executable="rl_controller_node",
    )
    actions = [simulation_launch, TimerAction(period=4.0, actions=[furuta_pendulum_rl_controller_node])]

    return LaunchDescription(actions)
