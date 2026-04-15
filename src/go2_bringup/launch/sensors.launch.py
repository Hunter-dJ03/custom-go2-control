"""Sensor subsystem: front camera (+ future LiDAR, ZED)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_arg = DeclareLaunchArgument(
        "front_camera", default_value="true", description="Launch Go2 front camera node"
    )
    iface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="enp2s0",
        description="Network interface connected to Go2 MCU",
    )

    camera_launch = os.path.join(
        get_package_share_directory("go2_camera"),
        "launch",
        "camera.launch.py",
    )

    return LaunchDescription(
        [
            camera_arg,
            iface_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "network_interface": LaunchConfiguration("network_interface"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("front_camera")),
            ),
        ]
    )
