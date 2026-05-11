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

**What frame are the points in?** Each `PointCloud2` stores `(x,y,z)` in the coordinate axes of whatever frame the **driver** put in `header.frame_id` on `/utlidar/cloud` (often something like `utlidar_lidar` — confirm with one message: `ros2 topic echo /utlidar/cloud --once` and read `header.frame_id`).

**Scale and offset** are applied **directly in that same gathered frame**: `p' = scale * p + offset` uses the message’s own `x,y,z` axes. The static TF (`radar` → `tf_child_frame`, etc.) only relates frames in **TF**; it does **not** rotate or re-express the point coordinates unless you add that separately.

**`use_input_frame_id`** (default `true`): the published cloud **keeps** the incoming `header.frame_id`, so the header always matches the frame the numbers are in. Set to `false` to force `output_frame_id` instead (use only if that name matches the same axes).

When `broadcast_radar_tf` is true (default), the node publishes a static transform on `/tf_static`: **`tf_parent_frame` → `tf_child_frame`** (default **`radar` → `utlidar_lidar`**). Set **`tf_child_frame`** to the **same string** as your real lidar `frame_id` so TF matches the cloud. Rotation about the parent **+Z** is **`tf_yaw_deg`** degrees (default **120**); translation is zero.

| Parameter | Default |
|-----------|---------|
| `input_topic` | `/utlidar/cloud` |
| `output_topic` | `/go2/lidar/points_calibrated` |
| `use_input_frame_id` | `true` |
| `output_frame_id` | `utlidar_lidar` (used only if `use_input_frame_id` is `false`) |
| `broadcast_radar_tf` | `true` |
| `tf_parent_frame` | `radar` |
| `tf_child_frame` | `utlidar_lidar` |
| `tf_yaw_deg` | `120.0` |
| `scale` | `1.0` |
| `offset_x`, `offset_y`, `offset_z` | `0.0` |

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
