"""Placeholder map -> odom TF (identity) and odom trajectory path for RViz.

Disable `map_odom_identity_tf` when another node publishes `map -> odom` (e.g. RTAB-Map),
or transforms will conflict with the static broadcaster on `/tf_static`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_odom_identity_tf_arg = DeclareLaunchArgument(
        'map_odom_identity_tf',
        default_value='true',
        description='If true, publish static identity map→odom (needed for RViz fixed_frame=map '
                    'until SLAM provides map→odom). Set false alongside rtabmap or any SLAM publishing map→odom.',
    )
    history_sec = DeclareLaunchArgument(
        'path_history_seconds',
        default_value='120.0',
        description='Sliding window (seconds) of odometry kept in /go2/path',
    )
    path_stride = DeclareLaunchArgument(
        'path_odom_stride',
        default_value='15',
        description='Append one pose to /go2/path every Nth odometry message (1 = every odom)',
    )
    return LaunchDescription(
        [
            map_odom_identity_tf_arg,
            history_sec,
            path_stride,
            Node(
                package='go2_localisation',
                executable='map_odom_tf_node',
                name='map_odom_tf_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('map_odom_identity_tf')),
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
                        'path_odom_stride': ParameterValue(
                            LaunchConfiguration('path_odom_stride'),
                            value_type=int,
                        ),
                        'max_path_poses': 50000,
                    },
                ],
            ),
        ]
    )
