# go2_slam

Simultaneous localisation and mapping.

## Layer

5 — Autonomy

## Status

**DEFERRED** — implement after sensor pipeline is validated (Phase 4).

## Purpose

Builds maps of unknown environments using LiDAR and IMU data.
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
