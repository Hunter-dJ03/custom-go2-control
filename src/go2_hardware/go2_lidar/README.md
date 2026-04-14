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
