#!/usr/bin/env python3

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    furuta_pendulum_controller_node = Node(
        package="furuta_pendulum_control_toolbox",
        executable="ct_mpc_controller_node",
        output="screen",
        emulate_tty=True,
    )

    actions = [
        furuta_pendulum_controller_node,
    ]

    return LaunchDescription(actions)
