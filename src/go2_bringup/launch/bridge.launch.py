"""Unitree DDS ↔ ROS bridge only (no robot model — use description.launch or phase1)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_bridge')
    default_joint_states_params = os.path.join(pkg_share, 'config', 'joint_states_throttle.yaml')
    default_tf_lf_params = os.path.join(pkg_share, 'config', 'tf_lf_relay.yaml')

    joint_states_throttle_arg = DeclareLaunchArgument(
        'joint_states_throttle',
        default_value='true',
        description='Relay /go2/joint_states to /go2/lf/joint_states at a lower rate for remote viz',
    )
    joint_states_throttle_params_arg = DeclareLaunchArgument(
        'joint_states_throttle_params_file',
        default_value=default_joint_states_params,
        description='YAML parameters for joint_states_throttle_node',
    )
    tf_lf_relay_arg = DeclareLaunchArgument(
        'tf_lf_relay',
        default_value='true',
        description='Relay /tf (no leg links) to /go2/lf/tf at a lower rate for remote viz',
    )
    tf_lf_relay_params_arg = DeclareLaunchArgument(
        'tf_lf_relay_params_file',
        default_value=default_tf_lf_params,
        description='YAML parameters for tf_lf_relay_node',
    )

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

    joint_states_throttle = Node(
        package='go2_bridge',
        executable='joint_states_throttle_node',
        name='joint_states_throttle_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_states_throttle')),
        parameters=[
            ParameterFile(LaunchConfiguration('joint_states_throttle_params_file'), allow_substs=True),
        ],
    )

    tf_lf_relay = Node(
        package='go2_bridge',
        executable='tf_lf_relay_node',
        name='tf_lf_relay_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('tf_lf_relay')),
        parameters=[
            ParameterFile(LaunchConfiguration('tf_lf_relay_params_file'), allow_substs=True),
        ],
    )

    return LaunchDescription([
        joint_states_throttle_arg,
        joint_states_throttle_params_arg,
        tf_lf_relay_arg,
        tf_lf_relay_params_arg,
        bridge,
        joint_states_throttle,
        tf_lf_relay,
    ])
