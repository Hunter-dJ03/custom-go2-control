# go2_slam

Simultaneous localisation and mapping.

## Layer

5 — Autonomy

## RTAB-Map (implemented)

Launch **after** `go2_bringup` (sensors + bridge) is running.

**TF ownership:** Phase 1 defaults **`map_odom_identity_tf:=false`** and **does not** publish Go2 `odom` → `base_link`. This SLAM launch runs ZED RGB-D odometry (`odom_source:=rgbd`, default) or ZED SDK tracking odometry (`odom_source:=zed`), rebases startup `z` from the one-shot `/go2/odom_seed` bridge topic, publishes the corrected `/go2/odom` plus TF `odom` → `base_link`, and lets RTAB-Map publish `map` → `odom`.

Nodes use ROS namespace **`rtabmap`** by default (`rtabmap_ns`): map and clouds appear as **`/rtabmap/map`**, **`/rtabmap/assembled_cloud`**, **`/rtabmap/rgbd_image`**, etc. Use **`rtabmap_ns:=`** empty only for legacy unprefixed topics.

```bash
# Default: LiDAR mapping with RTAB-Map rgbd_odometry + loop closure
ros2 launch go2_slam rtabmap.launch.py

# ZED SDK tracking odometry instead of rgbd_odometry (still rebases startup z)
ros2 launch go2_slam rtabmap.launch.py odom_source:=zed

# ZED-only mapping with ZED tracking odometry
ros2 launch go2_slam rtabmap.launch.py map_registration_source:=zed odom_source:=zed
```

Default LiDAR mode sets **Reg/Strategy = Visual+ICP**, so the camera provides odometry + loop closure while LiDAR contributes mapping geometry. ZED-primary mode uses **Reg/Strategy = 0** for RGB-D-only odometry, loop closure, and mapping.

`rgbd_sync` publishes `rtabmap_msgs/RGBDImage` on **`/rtabmap/rgbd_image`** (unless you set `rgbd_image_topic`). This launch always runs `rgbd_sync`; `rgbd_odometry` runs unless **`odom_source:=zed`**. The odometry TF is rebased once at startup so the initial `base_link` height in `odom` matches the first `/sportmodestate` pose instead of starting at `z=0`. With **`odom_source:=zed`**, keep **`zed_publish_odom_tf:=false`** on phase1 so `odom_rebase_node` owns `odom` → `base_link`.

Default **`qos:=2`** (best effort) matches **`go2_lidar`**/**`go2_bridge`** `SensorDataQoS` publishers; use **`qos:=1`** only if everything on the lidar RGB-D chain is reliable. Default ZED topics match **`MAKERSPACE/src/bringup/launch/odometry_launch.py`**: `/zed/zed_node/rgb/color/rect/image`, matching `camera_info`, depth `/zed/zed_node/depth/depth_registered`; **`rgbd_approx_sync`** defaults to **false** (exact sync). Set `rgbd_approx_sync:=true` if you need approximate sync and tune **`approx_rgbd_sync_max_interval`**.

Override **`zed_rgb_topic`**, **`zed_camera_info_topic`**, or **`zed_depth_topic`** if your ZED graph differs (e.g. older `image_rect_color` layout).

## ZED-only SLAM (`zed.launch.py`)

Strict copy of upstream **`rtabmap_examples/zed.launch.py`** (apt `ros-humble-rtabmap-examples`) — same shared `parameters` list and `remappings`, same nodes (`zed_camera.launch.py` + `rgbd_sync` + `rgbd_odometry` + `rtabmap` + `rtabmap_viz`). Two intentional deviations:

- Topics use the **modern** stereolabs `zed-ros2-wrapper` layout (`/zed/zed_node/rgb/color/rect/image` + matching `camera_info`); upstream still uses the legacy `/zed/zed_node/rgb/image_rect_color`.
- `wait_imu_to_init: False` (upstream is `True`). Stock `zed-ros2-wrapper` does **not** publish the `zed_camera_link -> zed_imu_link` static TF, so `True` blocks `rgbd_odometry` forever.

Topics are **hardcoded** to match upstream style — if you need a different ZED topic graph, edit the launch file (the per-topic launch args that used to live here have been removed for parity with upstream).

With phase1 (bridge, lidar, mount TF) — disable the duplicate ZED driver on phase1:

```bash
ros2 launch go2_bringup phase1.launch.py zed_camera:=false
ros2 launch go2_slam zed.launch.py camera_model:=zed2i
```

Use `use_zed_odometry:=true` to feed ZED's `/zed/zed_node/odom` into `rtabmap` instead of running `rgbd_odometry` here.

Default phase1 (`zed_camera:=true`) uses **`zed_override.yaml`** via sensors; the SLAM launch uses the upstream VGA tempfile override instead.

## Status

**In progress** — RTAB-Map launch is wired; tuning and outdoor tests remain.

## Purpose

Builds maps using LiDAR or ZED RGB-D while always using ZED RGB-D odometry.  
Publishes the map and the map → odom TF transform during active mapping.

Separated from `go2_localisation` to allow independent use:
- SLAM for exploration and map building
- Localisation for operating within a known map

## Subscriptions (planned)

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/lidar/points_filtered` | sensor_msgs/PointCloud2 | LiDAR input |
| `/go2/odom_seed` | geometry_msgs/PoseStamped | Startup height seed from the bridge |
| `/go2/odom` | nav_msgs/Odometry | Rebased visual odometry published by this stack |
| `/go2/imu/data` | sensor_msgs/Imu | IMU for inertial fusion |

## Publications (planned)

| Topic | Type |
|-------|------|
| `/go2/map` | nav_msgs/OccupancyGrid |
| TF: map → odom | tf2 |

## Candidate implementations

- LIO-SAM (LiDAR-inertial odometry and mapping)
- Point-LIO
- RTAB-Map

## Dependencies (planned)

- `sensor_msgs`, `nav_msgs`, `tf2_ros`
- SLAM framework package
