# Vendor copy of `rtabmap_examples/launch/zed.launch.py`, installed with `go2_slam`.
#
# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper (https://github.com/stereolabs/zed-ros2-wrapper).
#
# Run (after sourcing your workspace install):
#   ros2 launch go2_slam zed.launch.py camera_model:=zed2i
#
# RTAB-Map nodes (rgbd_sync, rgbd_odometry, rtabmap, rtabmap_viz) default to ROS
# namespace `rtabmap` (`rtabmap_ns`), matching `rtabmap.launch.py` so Foxglove
# whitelist `^/rtabmap($|/.*)` sees slam topics. Set `rtabmap_ns:=` empty for
# legacy unprefixed graph (ZED wrapper topics remain under `/zed/...` regardless).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile


def launch_setup(context: LaunchContext, *args, **kwargs):

    rtabmap_ns = LaunchConfiguration('rtabmap_ns').perform(context).strip().strip('/')

    zed_rgb = LaunchConfiguration('zed_rgb_topic').perform(context)
    zed_ci = LaunchConfiguration('zed_camera_info_topic').perform(context)
    zed_depth = LaunchConfiguration('zed_depth_topic').perform(context)
    zed_imu = LaunchConfiguration('zed_imu_topic').perform(context)
    zed_odom_topic = LaunchConfiguration('zed_odom_topic').perform(context)

    wait_imu = LaunchConfiguration('wait_imu_to_init').perform(context).strip().lower() in (
        'true', '1', 'yes')

    use_z_tf = LaunchConfiguration('use_zed_odometry').perform(context)

    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write(
            "---\n"
            "/**:\n"
            "    ros__parameters:\n"
            "        general:\n"
            "            grab_resolution: 'VGA'")

    rtabmap_base = {
        'frame_id': 'zed_camera_link',
        'subscribe_rgbd': True,
        'approx_sync': False,
    }
    rtabmap_viz_parameters = [dict(rtabmap_base)]
    if use_z_tf not in ("True", "true"):
        rtabmap_viz_parameters.append({'subscribe_odom_info': True})

    rgbd_odometry_parameters = (
        list(rtabmap_viz_parameters)
        + [{'wait_imu_to_init': wait_imu, 'always_check_imu_tf': wait_imu}])

    rgbd_sync_parameters = [{'approx_sync': False}]

    remappings = [('imu', zed_imu)]
    if use_z_tf in ("True", "true"):
        remappings.append(('odom', zed_odom_topic))

    zed_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py']),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'ros_params_override_path': zed_override_file.name,
            'publish_tf': LaunchConfiguration('use_zed_odometry'),
            'publish_map_tf': 'false'}.items())

    slam_nodes = [
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=rgbd_sync_parameters,
            remappings=[
                ('rgb/image', zed_rgb),
                ('rgb/camera_info', zed_ci),
                ('depth/image', zed_depth),
            ],
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=rgbd_odometry_parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_viz_parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=rtabmap_viz_parameters,
            remappings=remappings),
    ]

    if rtabmap_ns:
        return [
            zed_driver,
            GroupAction(
                actions=[PushRosNamespace(rtabmap_ns), *slam_nodes],
            ),
        ]
    return [zed_driver, *slam_nodes]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description=(
                'Use zed\'s computed odometry instead of using '
                'rtabmap\'s rgbd_odometry.')),

        DeclareLaunchArgument(
            'rtabmap_ns',
            default_value='rtabmap',
            description='ROS namespace for rtabmap nodes (/rtabmap/map, /rtabmap/rgbd_image, …). '
                        'Empty string disables PushRosNamespace (legacy global topics). '
                        'ZED driver stays under `/zed` unless overridden via zed_*_topic.'),
        DeclareLaunchArgument(
            'camera_model', default_value='',
            description=(
                "[REQUIRED] The model of the camera. Using a wrong camera model can disable "
                "camera features. Valid choices are: "
                "['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']")),
        DeclareLaunchArgument(
            'zed_rgb_topic',
            default_value='/zed/zed_node/rgb/color/rect/image',
            description=(
                'rectified RGB for rgbd_sync (modern zed-ros2-wrapper; '
                'override for legacy `/zed/zed_node/rgb/image_rect_color`).')),
        DeclareLaunchArgument(
            'zed_camera_info_topic',
            default_value='/zed/zed_node/rgb/color/rect/image/camera_info',
            description='CameraInfo matching zed_rgb_topic.'),
        DeclareLaunchArgument(
            'zed_depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
            description='registered depth aligned to rectified RGB.'),
        DeclareLaunchArgument(
            'zed_imu_topic',
            default_value='/zed/zed_node/imu/data',
            description='IMU topic for rgbd_odometry/rtabmap (when subscribed).'),
        DeclareLaunchArgument(
            'zed_odom_topic',
            default_value='/zed/zed_node/odom',
            description='ZED visual odometry topic when use_zed_odometry:=true.'),

        DeclareLaunchArgument(
            'wait_imu_to_init',
            default_value='false',
            description=(
                'rgbd_odometry only. Stock zed_ros2_wrapper does not publish zed_camera_link→zed_imu_link '
                'TF; leave **false** or rgbd_odometry never outputs /odom. Set true only if that TF exists.'),
        ),

        OpaqueFunction(function=launch_setup),
    ])
