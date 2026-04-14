# go2_localisation

Localisation against existing maps.

## Layer

5 — Autonomy

## Status

**DEFERRED** — implement after SLAM and sensor pipeline are validated (Phase 4).

## Purpose

Localises the robot within a previously built map. Publishes the
map → odom TF transform for downstream navigation.

Separated from `go2_slam`:
- **go2_slam** builds maps of unknown environments
- **go2_localisation** operates within a known map

Does NOT use Unitree's built-in localisation (requires expansion dock).
Fully custom implementation running on the AGX Orin.

## Subscriptions (planned)

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/lidar/points_filtered` | sensor_msgs/PointCloud2 | LiDAR input |
| `/go2/odom` | nav_msgs/Odometry | Odometry prior |
| `/go2/imu/data` | sensor_msgs/Imu | IMU for inertial fusion |

## Publications (planned)

| Topic | Type |
|-------|------|
| TF: map → odom | tf2 |

## Candidate implementations

- AMCL (Nav2 adaptive Monte Carlo localisation)
- RTAB-Map localisation mode

## Dependencies (planned)

- `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Localisation framework package
