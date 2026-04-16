"""Phase 1: robot model (go2_description) + Unitree bridge + optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz2 with go2_default config',
    )
    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value='true',
        description='Include sensors.launch.py (front camera, etc.)',
    )
    front_camera_arg = DeclareLaunchArgument(
        'front_camera',
        default_value='true',
        description='Start go2_camera when sensors is enabled',
    )
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp2s0',
        description='Interface for Go2 camera multicast (sensors)',
    )
    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='30',
        description='Camera rate when sensors are enabled',
    )
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG quality for compressed camera topic',
    )
    path_history_arg = DeclareLaunchArgument(
        'path_history_seconds',
        default_value='120.0',
        description='Sliding window (seconds) for /go2/path odometry trail',
    )
    path_odom_stride_arg = DeclareLaunchArgument(
        'path_odom_stride',
        default_value='15',
        description='Append to /go2/path every Nth odometry message',
    )

    desc_launch = os.path.join(
        get_package_share_directory('go2_description'),
        'launch',
        'description.launch.py',
    )
    bridge_launch = os.path.join(
        get_package_share_directory('go2_bringup'),
        'launch',
        'bridge.launch.py',
    )
    sensors_launch = os.path.join(
        get_package_share_directory('go2_bringup'),
        'launch',
        'sensors.launch.py',
    )
    localization_launch = os.path.join(
        get_package_share_directory('go2_localisation'),
        'launch',
        'localization.launch.py',
    )
    rviz_config = os.path.join(
        get_package_share_directory('go2_rviz'),
        'rviz',
        'go2_default.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(
        [
            rviz_arg,
            sensors_arg,
            front_camera_arg,
            network_interface_arg,
            target_fps_arg,
            jpeg_quality_arg,
            path_history_arg,
            path_odom_stride_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(desc_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    'path_history_seconds': LaunchConfiguration('path_history_seconds'),
                    'path_odom_stride': LaunchConfiguration('path_odom_stride'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    'front_camera': LaunchConfiguration('front_camera'),
                    'network_interface': LaunchConfiguration('network_interface'),
                    'target_fps': LaunchConfiguration('target_fps'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('sensors')),
            ),
            rviz_node,
        ]
    )
