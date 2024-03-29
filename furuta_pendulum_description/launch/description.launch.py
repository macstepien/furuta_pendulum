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


def load_yaml(package_name, directory_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, directory_name, file_name)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    pendulum_parameters = load_yaml(
        "furuta_pendulum_description", "config", "pendulum_parameters.yaml"
    )["/**"]["ros__parameters"]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furuta_pendulum_description"),
                    "urdf",
                    "furuta_pendulum.urdf.xacro",
                ]
            ),
            " m1:=",
            str(pendulum_parameters["m1"]),
            " m2:=",
            str(pendulum_parameters["m2"]),
            " l1:=",
            str(pendulum_parameters["l1"]),
            " l2:=",
            str(pendulum_parameters["l2"]),
            " L1:=",
            str(pendulum_parameters["L1"]),
            " L2:=",
            str(pendulum_parameters["L2"]),
            " J1:=",
            str(pendulum_parameters["J1"]),
            " J2:=",
            str(pendulum_parameters["J2"]),
            " b1:=",
            str(pendulum_parameters["b1"]),
            " b2:=",
            str(pendulum_parameters["b2"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    actions = [
        robot_state_pub_node,
    ]

    return LaunchDescription(actions)
