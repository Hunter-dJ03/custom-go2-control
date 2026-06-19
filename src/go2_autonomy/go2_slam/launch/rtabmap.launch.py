# RTAB-Map SLAM launch for the Unitree Go2.
#
# TF architecture this launch assumes:
#   map               (published here by rtabmap_slam)
#   └── odom          published by go2_bridge (/go2/odom)
#       └── base_link
#           └── sensors   (URDF + static TFs from go2_bringup/sensors.launch.py)
#
# This launch ONLY runs RTAB-Map. Sensors (ZED2i, Unitree L2 LiDAR, the lidar
# calibration node, and the base_link->zed_camera_link static TF) come from
# `go2_bringup` (`sensors.launch.py`). Odometry comes from `go2_bridge`
# (`/go2/odom` + `odom -> base_link` TF). No icp_odometry / rgbd_odometry node
# is started here -- doing so would create a second `odom -> base_link`
# publisher and break TF.
#
# go2_bridge stamps `/go2/imu` and `/go2/odom` with `now()` (host time); ZED uses camera/driver
# timestamps — enabling `odom_sensor_sync:=true` on top of skewed clocks can degrade TF/visual
# registration. Prefer `odom_sensor_sync:=auto` (default): off when RGB-D + external_odom TF is used.
# rtabmap_viz defaults `max_odom_update_rate` to **auto**: **0 Hz throttle** (= unlimited) whenever
# there is RGB-D plus external odom — otherwise viz throttled frames look for rgbd_odometry's
# OdomInfo, which does not exist in this launch, so the GUI skips updates (“no green features”).
#
# Upstream zed.launch.py runs rgbd_odometry for visual odometry; that publishes OdomInfo that
# rtabmap_viz overlays. Wheel odom TF + rgbd_sync has no VO node — overlays come from slam stats/map.
#
# `go2_lidar` uses `SensorDataQoS` — default `qos` below is **best_effort** to match deskew/rtabmap subs.
#
# When `external_odom_frame_id` is set (default `odom`), the existing logic
# below skips `icp_odometry` and lets `rtabmap_slam` publish `map -> odom`.
#
# RGB-D loop closure: set `rgbd_sync:=true` to run `rtabmap_sync/rgbd_sync` in-process.
# Defaults match `/media/dogbot/MAKERSPACE/src/bringup/launch/odometry_launch.py` for rgbd_sync
# (rgbd_odometry is not used — external odom from go2_bridge). Optional GUI: `rtabmap_viz:=true`.
#
# All slam nodes live under ROS namespace `/rtabmap` by default (`rtabmap_ns`, e.g. `/rtabmap/assembled_cloud`,
# `/rtabmap/map`). Foxglove: `topic_whitelist` includes `^/rtabmap($|/.*)`. Set `rtabmap_ns:=` empty to revert.
#
# Example (after sensors + bridge are up — disable phase1 placeholder map→odom when SLAM):
#   ros2 launch go2_bringup phase1.launch.py map_odom_identity_tf:=false &
#   ros2 launch go2_slam rtabmap.launch.py
#
# ZED RGB-D for visual loop closure (with LiDAR mapping, default):
#   ros2 launch go2_slam rtabmap.launch.py rgbd_sync:=true
#   (Reg/Strategy becomes Visual+ICP so the camera participates in loop closures, not ICP-on-LiDAR only.)
#
# ZED-primary mapping only (rgbd_sync implied; LiDAR deskew/assembler omitted; go2 odom unchanged):
#   ros2 launch go2_slam rtabmap.launch.py map_registration_source:=zed
#
# With RTAB-Map GUI viewer:
#   ros2 launch go2_slam rtabmap.launch.py rtabmap_viz:=true
#
# Override examples:
#   ros2 launch go2_slam rtabmap.launch.py voxel_size:=0.15 assembling_time:=0.3
#   ros2 launch go2_slam rtabmap.launch.py localization:=true

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id = LaunchConfiguration('frame_id')

    external_odom_frame_id = LaunchConfiguration('external_odom_frame_id').perform(context)

    rtabmap_ns = LaunchConfiguration('rtabmap_ns').perform(context).strip().strip('/')

    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id:
        if external_odom_frame_id:
            fixed_frame_id = external_odom_frame_id
        else:
            fixed_frame_from_imu = True
            fixed_frame_id = frame_id.perform(context) + "_stabilized"

    imu_topic = LaunchConfiguration('imu_topic')

    map_registration_source = (
        LaunchConfiguration('map_registration_source').perform(context).lower().strip()
    )
    zed_primary = map_registration_source == 'zed'

    rgbd_sync_user = LaunchConfiguration('rgbd_sync').perform(context).lower() == 'true'
    # ZED-primary mode always runs rgbd_sync (aligned with go2_bridge odom — no rgbd_odometry here).
    effective_rgbd_sync = zed_primary or rgbd_sync_user

    rgbd_image_topic_str = LaunchConfiguration('rgbd_image_topic').perform(context)
    rgbd_images_topic_str = LaunchConfiguration('rgbd_images_topic').perform(context)
    # When rgbd_sync runs, publish/consume RGBDImage here unless overridden (relative topic when namespaced).
    if effective_rgbd_sync and not rgbd_image_topic_str and not rgbd_images_topic_str:
        rgbd_image_topic_str = 'rgbd_image' if rtabmap_ns else '/rtabmap/rgbd_image'
    rgbd_image_used = rgbd_image_topic_str != '' or rgbd_images_topic_str != ''
    rgbd_cameras = 0 if rgbd_images_topic_str != '' else 1

    # rtabmap core Registration::Type: 0=Visual (zed.launch.py default via rgbd_odometry-less stack),
    # 1=ICP (LiDAR-only), 2=Visual+ICP. With LiDAR + rgbd_sync, strategy 1 *ignores imagery for loop
    # closures* — only voxelized scans register; ZED contributes little. Strategy 2 matches common
    # LiDAR+camera setups where the camera closes loops and LiDAR refines geometry/occupancy.
    if zed_primary:
        registration_strategy_str = '0'
    elif rgbd_image_used:
        registration_strategy_str = '2'
    else:
        registration_strategy_str = '1'

    odom_sync_arg = LaunchConfiguration('odom_sensor_sync').perform(context).strip().lower()
    if odom_sync_arg == 'auto':
        # Bridge odom timestamps often don't match camera; extra odom-vs-image correction can fail.
        odom_sensor_sync = not (rgbd_image_used and external_odom_frame_id)
    elif odom_sync_arg in ('true', '1', 'yes'):
        odom_sensor_sync = True
    else:
        odom_sensor_sync = False

    viz_odom_arg = LaunchConfiguration('rtabmap_viz_max_odom_update_rate').perform(context).strip().lower()
    if viz_odom_arg == 'auto':
        viz_max_odom_update_rate = 0.0 if (
            rgbd_image_used and external_odom_frame_id) else 10.0
    else:
        viz_max_odom_update_rate = float(viz_odom_arg)

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + "/deskewed"

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    qos_int = int(LaunchConfiguration('qos').perform(context).strip())

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'

    # Rule of thumb:
    max_correspondence_distance = voxel_size_value * 10.0

    # Approx sync on rtabmap: LiDAR+RGB-D needs merging; RGB-D-only keeps false (see rtabmap_examples zed.launch.py style).
    rtabmap_approx_sync = (not zed_primary) and rgbd_image_used

    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': qos_int,
        'approx_sync': rtabmap_approx_sync,
        'wait_for_transform': 0.2,
        # RTAB-Map's internal parameters are strings:
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }

    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'wait_imu_to_init': True,
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': fixed_frame_id,
        # RTAB-Map's internal parameters are strings:
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
        'Icp/CorrespondenceRatio': '0.01',
    }

    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': not external_odom_frame_id,
        'subscribe_scan_cloud': not zed_primary,
        # When external_odom_frame_id is set, rtabmap publishes map -> odom.
        'odom_frame_id': (external_odom_frame_id if external_odom_frame_id else ""),
        'odom_sensor_sync': odom_sensor_sync,
        # RTAB-Map's internal parameters are strings:
        'Rtabmap/DetectionRate': '0',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        # ZED-primary: pure visual (like upstream zed.launch.py). LiDAR+ZED: Vis+ICP for camera loop closure.
        'Reg/Strategy': registration_strategy_str,
        'Icp/CorrespondenceRatio': str(
            LaunchConfiguration('min_loop_closure_overlap').perform(context)
        ),

        # Grid basics
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '6.0',
        'Grid/RayTracing': 'true',

        # Ground and obstacle separation
        'Grid/MinGroundHeight': '-0.50',
        'Grid/MaxGroundHeight': '-0.2',
        'Grid/MinObstacleHeight': '0.15',
        'Grid/MaxObstacleHeight': '3.80',
        'Grid/MaxGroundAngle': '20',

        # Noise cleanup in the projected grid
        'Grid/NoiseFilteringRadius': '0.20',
        'Grid/NoiseFilteringMinNeighbors': '3',

        # Downsample and cluster filtering before projection
        'Grid/PreVoxelFiltering': str(voxel_size_value),
        'Grid/ClusterRadius': str(max(0.10, voxel_size_value * 2.0)),
        'Grid/MinClusterSize': '8',

        # Keep these conservative
        'Grid/GroundIsObstacle': 'false',
        'Grid/NormalsSegmentation': 'false',
        'Grid/FlatObstacleDetected': 'false',
    }

    remappings = [('imu', imu_topic)]
    if rgbd_image_used:
        if rgbd_cameras == 1:
            remappings.append(('rgbd_image', rgbd_image_topic_str))
        else:
            remappings.append(('rgbd_images', rgbd_images_topic_str))

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        # Delete the previous database (~/.ros/rtabmap.db) on each fresh mapping run.
        arguments.append('-d')

    slam_remappings = list(remappings) + [('gps/fix', LaunchConfiguration('gps_topic'))]
    if not zed_primary:
        slam_remappings.append(('scan_cloud', 'assembled_cloud'))

    use_rtabmap_viz = (
        LaunchConfiguration('rtabmap_viz').perform(context).lower() == 'true'
    )

    rtabmap_extra_params = {
        'subscribe_rgbd': rgbd_image_used,
        'rgbd_cameras': rgbd_cameras,
        'topic_queue_size': 40,
        'sync_queue_size': 40,
    }

    nodes = []

    # LiDAR mapping path only (default). ZED-primary uses RGB-D-only like rtabmap_examples/zed.launch.py.
    if not zed_primary:
        nodes.extend([
            Node(
                package='rtabmap_util', executable='lidar_deskewing', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'qos': qos_int,
                    'fixed_frame_id': fixed_frame_id,
                    'wait_for_transform': 0.2,
                    'slerp': deskewing_slerp,
                }],
                remappings=[
                    ('input_cloud', lidar_topic),
                ]),
            Node(
                package='rtabmap_util', executable='point_cloud_assembler', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'qos': qos_int,
                    'qos_odom': qos_int,
                    'assembling_time': LaunchConfiguration('assembling_time'),
                    'fixed_frame_id': (
                        external_odom_frame_id if external_odom_frame_id else ''),
                }],
                remappings=[('cloud', lidar_topic_deskewed)]),
        ])

    nodes.append(
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[
                shared_parameters,
                rtabmap_parameters,
                rtabmap_extra_params,
            ],
            remappings=slam_remappings,
            arguments=arguments),
    )

    if use_rtabmap_viz:
        viz_params = {'max_odom_update_rate': viz_max_odom_update_rate}
        nodes.append(
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                output='screen',
                parameters=[
                    shared_parameters,
                    rtabmap_parameters,
                    rtabmap_extra_params,
                    viz_params,
                ],
                remappings=slam_remappings,
            ),
        )

    # ZED RGB + depth + camera_info -> RGBDImage (Makerspace / zed.launch.py style;
    # ZED topic paths are zed_*_topic args — not rtabmap_examples image_rect_color defaults).
    if effective_rgbd_sync and rgbd_cameras == 1:
        zed_rgb = LaunchConfiguration('zed_rgb_topic').perform(context)
        zed_ci = LaunchConfiguration('zed_camera_info_topic').perform(context)
        zed_depth = LaunchConfiguration('zed_depth_topic').perform(context)
        approx_sync_rgbd = (
            LaunchConfiguration('rgbd_approx_sync').perform(context).lower() == 'true'
        )
        rgbd_sync_params = {
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'qos': qos_int,
            'qos_camera_info': qos_int,
            'subscribe_rgbd': True,
            'approx_sync': approx_sync_rgbd,
            'wait_imu_to_init': True,
            'subscribe_odom_info': True,
        }
        if approx_sync_rgbd:
            rgbd_sync_params['approx_sync_max_interval'] = float(
                LaunchConfiguration('approx_rgbd_sync_max_interval').perform(context)
            )
        nodes.insert(
            0,
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync',
                output='screen',
                parameters=[rgbd_sync_params],
                remappings=[
                    ('rgb/image', zed_rgb),
                    ('rgb/camera_info', zed_ci),
                    ('depth/image', zed_depth),
                    ('rgbd_image', rgbd_image_topic_str),
                ],
            ),
        )

    if not external_odom_frame_id:
        # Lidar odometry (only when NO external odom is provided).
        # For the Go2 we leave external_odom_frame_id=odom, so this branch is skipped
        # and we rely on go2_bridge's odom -> base_link.
        nodes.append(
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[shared_parameters, icp_odometry_parameters],
                remappings=remappings + [('scan_cloud', lidar_topic_deskewed)]))

    if fixed_frame_from_imu:
        # Create a stabilized base frame from IMU for lidar deskewing
        # (only when no external odom).
        nodes.append(
            Node(
                package='rtabmap_util', executable='imu_to_tf', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'base_frame_id': frame_id,
                    'wait_for_transform_duration': 0.001,
                }],
                remappings=[('imu/data', imu_topic)]))

    if rtabmap_ns:
        return [
            GroupAction(
                actions=[PushRosNamespace(rtabmap_ns), *nodes],
            )
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
        DeclareLaunchArgument(
            'map_registration_source',
            default_value='lidar',
            description='SLAM sensor mix: "lidar" = deskew + assemble LiDAR (+ optional rgbd_sync visual). '
                        '"zed" = RGB-D-only (like rtabmap_examples/zed.launch.py but with go2_bridge odom; '
                        'no rgbd_odometry / no LiDAR assembler).'),

        DeclareLaunchArgument(
            'rgbd_sync',
            default_value='false',
            description='Run rtabmap_sync/rgbd_sync for ZED inputs. Forced on when map_registration_source:=zed '
                        '(use zed_*_topic for paths; defaults match zed_wrapper rgb/color/rect + depth_registered).'),

        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch rtabmap_viz (RTAB-Map GUI); uses the same remap/subscriptions as rtabmap.'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated clock.'),

        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description='Base frame of the robot (Go2: base_link).'),

        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description='Fixed frame used for lidar deskewing. If empty, falls back to '
                        'external_odom_frame_id (preferred) or "<frame_id>_stabilized" '
                        '(when icp_odometry is run).'),

        DeclareLaunchArgument(
            'external_odom_frame_id', default_value='odom',
            description='External odometry frame (e.g. published by go2_bridge as '
                        '`odom -> base_link`). When non-empty, icp_odometry is NOT '
                        'launched and rtabmap publishes `map -> <external_odom_frame_id>`.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization mode (load existing rtabmap.db without overwriting).'),

        DeclareLaunchArgument(
            'rtabmap_ns',
            default_value='rtabmap',
            description='ROS namespace for slam nodes (/rtabmap/map, /rtabmap/assembled_cloud, …). '
                        'Empty string disables PushRosNamespace (legacy global topics).'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/go2/lidar/points_calibrated',
            description='Lidar PointCloud2 topic. Defaults to the calibrated Unitree L2 '
                        'cloud published by go2_lidar/lidar_calibration_node '
                        '(frame_id `utlidar_lidar`).'),

        DeclareLaunchArgument(
            'imu_topic', default_value='/go2/imu',
            description='IMU topic (published by go2_bridge in `base_link`). Only used '
                        'when external_odom_frame_id is empty (icp_odometry / imu_to_tf).'),

        DeclareLaunchArgument(
            'gps_topic', default_value='/gps/fix',
            description='GPS topic (unused on the Go2 unless GNSS is added).'),

        DeclareLaunchArgument(
            'rgbd_image_topic', default_value='',
            description='rtabmap_msgs/RGBDImage for rtabmap. When empty and rgbd_sync is used '
                        '(or map_registration_source:=zed), defaults to `rgbd_image` under '
                        'rtabmap_ns (normally /rtabmap/rgbd_image), or `/rtabmap/rgbd_image` if '
                        'rtabmap_ns is unset.'),

        DeclareLaunchArgument(
            'zed_rgb_topic',
            default_value='/zed/zed_node/rgb/color/rect/image',
            description='RGB image for rgbd_sync (Makerspace bringup: rgb/color/rect/image).'),
        DeclareLaunchArgument(
            'zed_camera_info_topic',
            default_value='/zed/zed_node/rgb/color/rect/image/camera_info',
            description='CameraInfo for rgbd_sync.'),
        DeclareLaunchArgument(
            'zed_depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
            description='Depth image for rgbd_sync (aligned to RGB).'),
        DeclareLaunchArgument(
            'rgbd_approx_sync',
            default_value='false',
            description='If true, use approximate RGB/depth sync (needs '
                        'approx_rgbd_sync_max_interval). Makerspace odometry_launch uses false.'),
        DeclareLaunchArgument(
            'approx_rgbd_sync_max_interval',
            default_value='0.1',
            description='Only used when rgbd_approx_sync:=true.'),

        DeclareLaunchArgument(
            'rgbd_images_topic', default_value='',
            description='RGBD images topic (ignored if empty, overrides "rgbd_image_topic" '
                        'if set). Output of rtabmap_sync rgbdx_sync node.'),

        DeclareLaunchArgument(
            'odom_sensor_sync',
            default_value='auto',
            description='Passed to slam + rtabmap_viz. '
                        '`auto`=off when RGB-D is used alongside external_odom TF (recommended for '
                        'go2_bridge stamped `now()` + ZED timestamps). Override true/false to force.',
        ),

        DeclareLaunchArgument(
            'rtabmap_viz_max_odom_update_rate',
            default_value='auto',
            description='Maximum odometry/visualization rate for rtabmap_viz (Hz); 0 = unlimited. '
                        '`auto` uses 0 when RGB-D + external_odom TF (otherwise viz throttled frames '
                        'expect rgbd_odometry OdomInfo, which this launch does not run).'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.2',
            description='Voxel size (m) of the downsampled lidar point cloud. '
                        'Indoor: 0.1-0.3, outdoor: 0.5+. Default tuned for Go2 indoor.'),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='Minimum scan overlap percentage to accept a loop closure.'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',
            description='Expected lidar frame rate (only used by icp_odometry).'),

        DeclareLaunchArgument(
            'assembling_time', default_value='0.2',
            description='How much time (sec) we assemble lidar scans before sending them '
                        'to the mapping node.'),

        DeclareLaunchArgument(
            'deskewing_slerp', default_value='true',
            description='Use fast slerp interpolation between first and last stamps of '
                        'the scan for deskewing. Less accurate than per-point TF '
                        'lookups but a lot faster.'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='Reliability tier for lidar_deskewing/point_cloud_assembler/rtabmap (and rgbd_sync '
                        'when used). Prefer **2=best_effort** for go2_bridge + go2_lidar '
                        '`SensorDataQoS` publishers. Use 1=reliable if your sensors publish reliable.'),

        OpaqueFunction(function=launch_setup),
        ]
    )
