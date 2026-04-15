"""Load Go2 URDF and run robot_state_publisher (no bridge).

go2_bridge publishes JointState on /go2/joint_states; robot_state_publisher
defaults to joint_states -> /joint_states, so we remap to the bridge topic.
(IMU / odometry / battery are separate: configure RViz or other tools to use
/go2/imu, /go2/odom, /go2/battery — not affected by this remap.)
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2_description')
    xacro_path = os.path.join(share, 'urdf', 'go2.urdf.xacro')
    robot_xml = subprocess.check_output(['xacro', xacro_path], text=True)

    joint_states_topic = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/go2/joint_states',
        description='Topic robot_state_publisher subscribes to for JointState',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_xml}],
        remappings=[
            ('joint_states', LaunchConfiguration('joint_states_topic')),
        ],
    )

    return LaunchDescription([joint_states_topic, rsp])
