"""Sensor subsystem: front camera, ZED2i, (+ future LiDAR)."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    zed_camera_arg = DeclareLaunchArgument(
        "zed_camera",
        default_value="true",
        description="Launch ZED2i (zed_wrapper zed_camera.launch.py) and base_link→zed_camera_link static TF",
    )
    zed_publish_odom_tf_arg = DeclareLaunchArgument(
        "zed_publish_odom_tf",
        default_value="false",
        description="If true, ZED publishes odom→zed_camera_link TF (conflicts with robot odom if also used)",
    )
    zed_publish_map_tf_arg = DeclareLaunchArgument(
        "zed_publish_map_tf",
        default_value="false",
        description="If true, ZED publishes map→odom TF (requires zed_publish_odom_tf)",
    )

    camera_launch = os.path.join(
        get_package_share_directory("go2_camera"),
        "launch",
        "camera.launch.py",
    )

    zed_share = get_package_share_directory("zed_wrapper")
    zed_launch = os.path.join(zed_share, "launch", "zed_camera.launch.py")
    zed_xacro = os.path.join(zed_share, "urdf", "zed_descr.urdf.xacro")

    pitch_rad = math.radians(15.0)

    zed_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_camera_base_tf",
        arguments=[
            "--x",
            "0.2776",
            "--y",
            "0.0",
            "--z",
            "0.09977",
            "--roll",
            "0.0",
            "--pitch",
            str(pitch_rad),
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "zed_camera_link",
        ],
        condition=IfCondition(LaunchConfiguration("zed_camera")),
    )

    zed_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_tf": LaunchConfiguration("zed_publish_odom_tf"),
            "publish_map_tf": LaunchConfiguration("zed_publish_map_tf"),
            "xacro_path": zed_xacro,
            # After YAML + launch dict; keeps UTM↔map TF off unless you use GNSS fusion.
            "param_overrides": "gnss_fusion.publish_utm_tf:=false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("zed_camera")),
    )

    return LaunchDescription(
        [
            camera_arg,
            iface_arg,
            target_fps_arg,
            jpeg_quality_arg,
            zed_camera_arg,
            zed_publish_odom_tf_arg,
            zed_publish_map_tf_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "network_interface": LaunchConfiguration("network_interface"),
                    "target_fps": LaunchConfiguration("target_fps"),
                    "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("front_camera")),
            ),
            zed_base_tf,
            zed_include,
        ]
    )
