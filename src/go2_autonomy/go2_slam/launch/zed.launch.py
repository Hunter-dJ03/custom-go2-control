# Adapted from rtabmap_examples/zed.launch.py (apt: ros-humble-rtabmap-examples).
# Same nodes, same shared `parameters` list, same `remappings` list, same flow.
# Two intentional deviations from upstream:
#   1. Topics updated to the modern stereolabs zed-ros2-wrapper layout
#      (`rgb/color/rect/image` + matching `camera_info`). Upstream still uses the
#      legacy `/zed/zed_node/rgb/image_rect_color` + `/zed/zed_node/rgb/camera_info`.
#   2. `wait_imu_to_init: False`. Stock zed_ros2_wrapper does NOT publish the
#      `zed_camera_link -> zed_imu_link` static TF ("Broadcast IMU TF [not for ZED]:
#      false"); keeping the upstream `True` blocks `rgbd_odometry` forever. Flip to
#      `True` only if you publish that static TF separately.
#
# Requirements:
#   A ZED camera.
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper).
#
# Example (Go2 ZED2i; disable the bringup ZED so it doesn't double-launch):
#   $ ros2 launch go2_bringup phase1.launch.py zed_camera:=false
#   $ ros2 launch go2_slam zed.launch.py camera_model:=zed2i

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile

parameters = []
remappings = []


def launch_setup(context: LaunchContext, *args, **kwargs):

    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n" +
                                "/**:\n" +
                                "    ros__parameters:\n" +
                                "        general:\n" +
                                "            grab_resolution: 'VGA'")

    parameters = [{'frame_id': 'zed_camera_link',
                   'subscribe_rgbd': True,
                   'approx_sync': False,
                   'wait_imu_to_init': False}]

    remappings = [('imu', '/zed/zed_node/imu/data')]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})

    return [
        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),
                '/zed_camera.launch.py']),
            launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                              'ros_params_override_path': zed_override_file.name,
                              'publish_tf': LaunchConfiguration('use_zed_odometry'),
                              'publish_map_tf': 'false'}.items(),
        ),

        # Sync rgb/depth/camera_info together
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/image/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=parameters,
            remappings=remappings),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)
    ]


def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description='Use zed\'s computed odometry instead of using rtabmap\'s odometry.'),

        DeclareLaunchArgument(
            'camera_model', default_value='',
            description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),

        OpaqueFunction(function=launch_setup)
    ])
