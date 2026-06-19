# go2_slam

Simultaneous localisation and mapping.

## Layer

5 — Autonomy

## RTAB-Map (implemented)

Launch **after** `go2_bringup` (sensors + bridge) is running.

**TF:** Phase 1’s default **`map_odom_identity_tf:=true`** starts a **static identity** `map` → `odom` placeholder. Running RTAB-Map alongside that **locks** loop-closure corrections. Use **`ros2 launch go2_bringup phase1.launch.py … map_odom_identity_tf:=false`** (or omit `localization` if split) so only RTAB‑Map publishes `map` → `odom`.

Nodes use ROS namespace **`rtabmap`** by default (`rtabmap_ns`): map and clouds appear as **`/rtabmap/map`**, **`/rtabmap/assembled_cloud`**, **`/rtabmap/rgbd_image`**, etc. Use **`rtabmap_ns:=`** empty only for legacy unprefixed topics.

```bash
# LiDAR-only mapping
ros2 launch go2_slam rtabmap.launch.py

# LiDAR + ZED RGB-D for visual loop closure (rgbd_sync node is started inside this launch)
ros2 launch go2_slam rtabmap.launch.py rgbd_sync:=true
```

That combination sets **Reg/Strategy = Visual+ICP** (upstream `zed.launch.py` is visual‑only **Reg/Strategy=0** without LiDAR; LiDAR‑only mapping here uses **ICP only**).

`rgbd_sync:=true` publishes `rtabmap_msgs/RGBDImage` on **`/rtabmap/rgbd_image`** (unless you set `rgbd_image_topic`). **Defaults:** **`odom_sensor_sync:=auto`** turns off RGB/odom time-compensation whenever RGB‑D runs with **`external_odom_frame_id`** (recommended with `go2_bridge` stamping `now()` vs ZED image stamps). **`rtabmap_viz_max_odom_update_rate:=auto`** sets **`0`** (= no throttle) in that same RGB‑D+external‑odom case — otherwise **`rtabmap_viz`** throttled frames skip updates unless **`rgbd_odometry`** publishes **`OdomInfo`** (we do **not** run that node to avoid conflicting `odom` TF). You still map and close loops inside **`rtabmap`**; VO-style green feature overlays behave like **`zed.launch.py`** only when a separate **`rgbd_odometry`** is running.

Default **`qos:=2`** (best effort) matches **`go2_lidar`**/**`go2_bridge`** `SensorDataQoS` publishers; use **`qos:=1`** only if everything on the lidar RGB-D chain is reliable. Default ZED topics match **`MAKERSPACE/src/bringup/launch/odometry_launch.py`**: `/zed/zed_node/rgb/color/rect/image`, matching `camera_info`, depth `/zed/zed_node/depth/depth_registered`; **`rgbd_approx_sync`** defaults to **false** (exact sync). Set `rgbd_approx_sync:=true` if you need approximate sync and tune **`approx_rgbd_sync_max_interval`**.

Override **`zed_rgb_topic`**, **`zed_camera_info_topic`**, or **`zed_depth_topic`** if your ZED graph differs (e.g. older `image_rect_color` layout).

## Status

**In progress** — RTAB-Map launch is wired; tuning and outdoor tests remain.

## Purpose

Builds maps using LiDAR (and optionally ZED RGB-D for loop closure).  
Publishes the map and the map → odom TF transform during active mapping.

Separated from `go2_localisation` to allow independent use:
- SLAM for exploration and map building
- Localisation for operating within a known map

## Subscriptions (planned)

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/lidar/points_filtered` | sensor_msgs/PointCloud2 | LiDAR input |
| `/go2/odom` | nav_msgs/Odometry | Odometry prior |
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
