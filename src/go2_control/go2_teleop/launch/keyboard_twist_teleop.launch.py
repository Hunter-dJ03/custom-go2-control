# Copyright 2025 go2_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Keyboard teleop + cmd_vel to Sport Move. Requires go2_bridge on /go2/sport_api_call."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_teleop")
    config = os.path.join(pkg, "config", "teleop_keyboard.yaml")
    return LaunchDescription(
        [
            Node(
                package="go2_teleop",
                executable="keyboard_teleop_node",
                name="go2_keyboard_teleop",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="go2_motion",
                executable="cmd_vel_to_sport_node",
                name="cmd_vel_to_sport_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
