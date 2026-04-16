"""Foxglove WebSocket bridge for Foxglove Studio (replaces rosbridge for Studio-native protocol)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_bringup')
    default_params = os.path.join(pkg_share, 'config', 'foxglove_phase1.yaml')

    port = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='TCP port for the Foxglove WebSocket server',
    )
    address = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Bind address (use 127.0.0.1 for local-only)',
    )
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='ROS 2 YAML parameter file for foxglove_bridge (topic_whitelist, qos, …)',
    )

    # ParameterFile allows overriding port/address from launch without editing YAML.
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            ParameterFile(LaunchConfiguration('params_file'), allow_substs=True),
            {'port': LaunchConfiguration('port'), 'address': LaunchConfiguration('address')},
        ],
    )

    return LaunchDescription([port, address, params_file, foxglove_node])
