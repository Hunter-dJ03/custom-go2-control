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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(desc_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
            ),
            rviz_node,
        ]
    )
