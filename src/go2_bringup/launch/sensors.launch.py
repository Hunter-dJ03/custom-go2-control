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
    target_fps_arg = DeclareLaunchArgument(
        "target_fps",
        default_value="30",
        description="Camera publishing rate passed to go2_camera",
    )
    jpeg_quality_arg = DeclareLaunchArgument(
        "jpeg_quality",
        default_value="80",
        description="JPEG quality for /go2/front_camera/image_raw/compressed",
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
            target_fps_arg,
            jpeg_quality_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "network_interface": LaunchConfiguration("network_interface"),
                    "target_fps": LaunchConfiguration("target_fps"),
                    "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("front_camera")),
            ),
        ]
    )
