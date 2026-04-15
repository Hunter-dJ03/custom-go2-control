"""Unitree DDS ↔ ROS bridge only (no robot model — use description.launch or phase1)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='go2_bridge',
        executable='go2_bridge_node',
        name='go2_bridge_node',
        output='screen',
        parameters=[
            {
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'sport_mode_state_topic': '/sportmodestate',
                'low_state_topic': '/lowstate',
            }
        ],
    )

    return LaunchDescription([bridge])
