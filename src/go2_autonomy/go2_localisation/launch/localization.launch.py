"""Placeholder map -> odom TF (identity) and odom trajectory path for RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    history_sec = DeclareLaunchArgument(
        'path_history_seconds',
        default_value='120.0',
        description='Sliding window (seconds) of odometry kept in /go2/path',
    )
    return LaunchDescription(
        [
            history_sec,
            Node(
                package='go2_localisation',
                executable='map_odom_tf_node',
                name='map_odom_tf_node',
                output='screen',
                parameters=[
                    {
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                    },
                ],
            ),
            Node(
                package='go2_localisation',
                executable='odom_path_history_node',
                name='odom_path_history_node',
                output='screen',
                parameters=[
                    {
                        'odom_topic': '/go2/odom',
                        'path_topic': '/go2/path',
                        'path_history_seconds': ParameterValue(
                            LaunchConfiguration('path_history_seconds'),
                            value_type=float,
                        ),
                        'max_path_poses': 50000,
                    },
                ],
            ),
        ]
    )
