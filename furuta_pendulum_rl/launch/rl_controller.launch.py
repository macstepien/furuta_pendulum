#!/usr/bin/env python3

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    furuta_pendulum_rl_controller_node = Node(
        package="furuta_pendulum_rl",
        executable="rl_controller_node",
    )
    actions = [furuta_pendulum_rl_controller_node]

    return LaunchDescription(actions)
