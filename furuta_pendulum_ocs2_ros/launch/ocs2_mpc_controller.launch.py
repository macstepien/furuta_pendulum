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

    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("furuta_pendulum_ocs2_ros"),
            "config",
            "controller.yaml",
        ]
    )


    mpc_mrt_node = Node(
        package="furuta_pendulum_ocs2_ros",
        executable="ocs2_mpc_controller_node",
        parameters=[controller_params],
    )

    actions = [
        mpc_mrt_node
    ]

    return LaunchDescription(actions)
