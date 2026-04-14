# go2_sensors

Sensor processing pipeline for the Unitree Go2's LiDAR, cameras, and additional sensors. Handles point cloud filtering, 2D scan projection, image rectification, and static TF broadcasting.

## Planned Nodes

### `lidar_processor_node`

Filters 3D point clouds by range and height, and optionally projects a 2D `LaserScan` slice for use with Nav2 and SLAM.

### `camera_processor_node`

Applies calibration-based undistortion to the front camera image stream.

### `sensor_tf_node`

Broadcasts static transforms between `base_link` and each sensor frame.

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | Raw 3D point cloud from the LiDAR |
| `/go2/camera/front/image_raw` | `sensor_msgs/Image` | Raw image from the front camera |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/go2/lidar/scan` | `sensor_msgs/LaserScan` | 2D scan projected from the 3D cloud |
| `/go2/lidar/points_filtered` | `sensor_msgs/PointCloud2` | Height- and range-filtered point cloud |
| `/go2/camera/front/image_rect` | `sensor_msgs/Image` | Undistorted front camera image |

## Static TF

The `sensor_tf_node` publishes the following static transforms:

| Parent | Child | Description |
|--------|-------|-------------|
| `base_link` | `lidar_link` | LiDAR mount position |
| `base_link` | `front_camera_link` | Front camera mount position |
| `base_link` | `imu_link` | IMU position |
| `base_link` | `zed_camera_link` | ZED stereo camera mount position |

Transform values will be populated once physical measurements or URDF data are finalized.

## ZED Integration

A placeholder is reserved for the ZED stereo camera (`zed_camera_link` frame). The `zed_ros2_wrapper` package will be launched separately and is not managed by this package. This package only provides the static TF from `base_link` to `zed_camera_link`.

## Parameters

### `lidar_processor_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | `double` | `0.1` | Minimum valid range (m) |
| `max_range` | `double` | `30.0` | Maximum valid range (m) |
| `min_height` | `double` | `-0.3` | Minimum height passthrough (m, relative to sensor) |
| `max_height` | `double` | `2.0` | Maximum height passthrough (m, relative to sensor) |
| `publish_2d_scan` | `bool` | `true` | Whether to publish a projected 2D LaserScan |
| `scan_height` | `double` | `0.0` | Height of the 2D scan slice (m, relative to sensor) |

### `camera_processor_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `calibration_file` | `string` | `""` | Path to camera calibration YAML |
| `undistort` | `bool` | `false` | Enable image undistortion |

### `sensor_tf_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_frame` | `string` | `"lidar_link"` | Frame ID for the LiDAR |
| `front_camera_frame` | `string` | `"front_camera_link"` | Frame ID for the front camera |
| `imu_frame` | `string` | `"imu_link"` | Frame ID for the IMU |
