# go2_localisation

Localisation against existing maps.

## Layer

5 — Autonomy

## Status

**DEFERRED** — full localisation (AMCL / RTAB-Map, etc.) comes after SLAM and sensor validation (Phase 4).

**Shipped now:** `map_odom_tf_node` publishes a **static identity** `map` → `odom` so the TF tree is `map` → `odom` → `base_link` (with `odom` → `base_link` from `go2_bridge`). RViz fixed frame can be `map`. Replace this node when SLAM publishes a real `map` → `odom` transform.

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

## Current nodes

| Executable | Role |
|------------|------|
| `map_odom_tf_node` | Static TF `map` → `odom` (identity). |
| `odom_path_history_node` | Subscribes to `/go2/odom`, publishes **`/go2/path`** (`nav_msgs/Path`): trajectory in the **odom** frame over a sliding time window (for RViz). |

**`map_odom_tf_node` parameters:** `map_frame` (default `map`), `odom_frame` (default `odom`).

**`odom_path_history_node` parameters:** `odom_topic`, `path_topic`, `path_history_seconds` (default `120.0`), `max_path_poses` (default `50000`, safety cap), `path_odom_stride` (default `15` — append only every Nth odometry message; `1` = every message). **`/go2/path` is published only when a pose is appended** (after stride), not on every odometry callback; the path publisher uses a shallow QoS history to limit backlog.

**Launch:** `localization.launch.py` and `phase1.launch.py` accept `path_history_seconds` and `path_odom_stride` (e.g. `path_odom_stride:=10`).

**Note:** Time-based pruning of old poses still runs every odometry message, but RViz will not see those removals until the next publish (next append). If the robot is stationary for a long time, the drawn trail may lag until motion resumes; use `path_odom_stride:=1` or add a separate publish-on-prune mode if you need that.

## Candidate implementations

- AMCL (Nav2 adaptive Monte Carlo localisation)
- RTAB-Map localisation mode

## TODO

- [ ] **Non-EKF localisation path:** Prefer the standard TF split before adding `robot_localisation` or another EKF: publish **`map` → `odom`** from exteroceptive SLAM / localisation (e.g. RTAB-Map with ZED), keep Unitree **`odom` → `base_link`** from `/go2/odom` / bridge, and validate **`map` → `odom` → `base_link`** composition in RViz. Only introduce sensor fusion EKF/UKF if TF + SLAM priors are insufficient for Nav2 or a specific downstream requirement.

## Dependencies (planned)

- `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Localisation framework package
