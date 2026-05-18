# go2_lidar

LiDAR data processing for the Unitree L2 LiDAR.

## Layer

4 — Sensors

## Purpose

Processes raw PointCloud2 data from the Unitree L2 LiDAR into formats
suitable for downstream SLAM and navigation:
- 2D LaserScan extraction (height slice for Nav2 costmaps)
- Voxel grid downsampling for SLAM
- Range filtering

## Nodes

### lidar_calibration_node (C++, `go2_lidar`)

**What frame are the points in?** Each `PointCloud2` stores `(x,y,z)` in the frame named in `header.frame_id` on the input topic. On the Go2 stack this is commonly:

- **`/utlidar/cloud_deskewed`** → `frame_id` is typically **`odom`** (deskewed cloud in the odometry frame).
- **`/utlidar/cloud`** → `frame_id` is typically **`utlidar_lidar`** (sensor frame).

Confirm with `ros2 topic echo <input_topic> --once` and read `header.frame_id`.

**`calibration_mode`** (string, default `cartesian`):

- **`cartesian`**: `p' = scale * p + offset` in the message axes. `range_scale` / `range_offset_m` are ignored.
- **`spherical`**: Same azimuth/elevation from the cloud origin; only range changes: `r = ‖p‖`, `r' = max(0, range_scale * r + range_offset_m)`, `p' = (r'/r) * p`. `scale` and `offset_*` are ignored.

The static TF (`tf_parent_frame` → `tf_child_frame`) only affects **TF**; it does **not** change point coordinates. If you use deskewed clouds in **`odom`**, keep **`broadcast_radar_tf`** false unless you intentionally publish a transform that matches that frame (see shipped `lidar_calibration.yaml`).

**`min_range_m`** (double, default `0.0`): drops returns whose raw range `r = ‖p‖` from the cloud origin is below this threshold, **before** any calibration. `0.0` disables. On `/utlidar/cloud` (sensor frame) this is a true sensor-range cull; on `/utlidar/cloud_deskewed` (odom frame) it is distance from the odom origin and only useful at the start of a run.

**`use_input_frame_id`** (default `true`): the published cloud **keeps** the incoming `header.frame_id`. Set to `false` to force `output_frame_id` (must match the same axes as the point data).

When `broadcast_radar_tf` is true, the node publishes a static transform on `/tf_static`: **`tf_parent_frame` → `tf_child_frame`**. Rotation about the parent **+Z** is **`tf_yaw_deg`**; translation is zero.

| Parameter | Default |
|-----------|---------|
| `input_topic` | `/utlidar/cloud_deskewed` |
| `output_topic` | `/go2/lidar/points_calibrated` |
| `use_input_frame_id` | `true` |
| `output_frame_id` | `odom` (used only if `use_input_frame_id` is `false`; must match deskewed axes if used) |
| `broadcast_radar_tf` | `false` |
| `tf_parent_frame` | `radar` |
| `tf_child_frame` | `utlidar_lidar` |
| `tf_yaw_deg` | `0.0` |
| `calibration_mode` | `cartesian` or `spherical` |
| `scale` | `1.0` (cartesian only) |
| `offset_x`, `offset_y`, `offset_z` | `0.0` (cartesian only) |
| `range_scale`, `range_offset_m` | `1.0`, `0.0` (spherical only; meters along the ray) |
| `min_range_m` | `0.0` (drop returns with raw r < this; 0 disables) |

Build `go2_lidar` from the **`custom-go2-control`** workspace root (where `install/` is produced), then source **`custom-go2-control/install/setup.bash`** (or `source .../setup_env.bash` from that repo). Using `colcon build` only under `unitree_ws` without this overlay will not refresh the binary `ros2 run` uses.

Run: `ros2 run go2_lidar lidar_calibration_node --ros-args --params-file $(ros2 pkg prefix go2_lidar)/share/go2_lidar/config/lidar_calibration.yaml`

### go2_lidar_node

**Subscriptions:**

| Topic | Type |
|-------|------|
| `/go2/lidar/points` | sensor_msgs/PointCloud2 |

**Publications:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/lidar/scan` | sensor_msgs/LaserScan | 2D scan for Nav2 |
| `/go2/lidar/points_filtered` | sensor_msgs/PointCloud2 | Filtered cloud for SLAM |

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `scan_height_min` | -0.1 | 2D scan slice lower bound (m) |
| `scan_height_max` | 0.3 | 2D scan slice upper bound (m) |
| `voxel_size` | 0.05 | Voxel filter resolution (m) |
| `range_min` | 0.15 | Minimum range filter (m) |
| `range_max` | 30.0 | Maximum range filter (m) |

## Dependencies

- `sensor_msgs`
- `pcl_conversions`, `pcl_ros`
- `rclcpp`
