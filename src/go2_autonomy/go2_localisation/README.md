# go2_localisation

Localisation against existing maps.

## Layer

5 — Autonomy

## Status

**DEFERRED** — full localisation (AMCL / RTAB-Map, etc.) comes after SLAM and sensor validation (Phase 4).

**Shipped now:** `map_odom_tf_node` (optional via **`map_odom_identity_tf`** in launch) publishes a **static identity** `map` → `odom` placeholder, `odom_path_history_node` turns `/go2/odom` into `/go2/path`, and `odom_rebase_node` rebases RTAB-Map visual odometry so the startup `z` comes from the bridge's one-shot `/go2/odom_seed` pose instead of starting at ground level. Keep **`map_odom_identity_tf:=false`** whenever SLAM is active so RTAB-Map can own `map` → `odom`.

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
| `odom_rebase_node` | Applies the startup `z` seed from `/go2/odom_seed` to raw visual odometry and republishes corrected `/go2/odom` plus `odom` → `base_link`. |

**`map_odom_tf_node` parameters:** `map_frame` (default `map`), `odom_frame` (default `odom`).

**`odom_path_history_node` parameters:** `odom_topic`, `path_topic`, `path_history_seconds` (default `120.0`), `max_path_poses` (default `50000`, safety cap), `path_odom_stride` (default `15` — append only every Nth odometry message; `1` = every message). **`/go2/path` is published only when a pose is appended** (after stride), not on every odometry callback; the path publisher uses a shallow QoS history to limit backlog.

**`odom_rebase_node` parameters:** `raw_odom_topic`, `output_odom_topic`, `seed_pose_topic`, `odom_frame`, `base_frame`, `publish_tf`, `use_host_time`, `transform_input_to_base` (compose `zed_camera_link` → `base_link` before rebasing z; used when `odom_source:=zed`).

**Launch:** `localization.launch.py` and `phase1.launch.py` accept `path_history_seconds`, `path_odom_stride`, and **`map_odom_identity_tf`** (default **`false`**; set **`true`** for a static identity `map` → `odom` placeholder when SLAM is not running).

**Note:** Time-based pruning of old poses still runs every odometry message, but RViz will not see those removals until the next publish (next append). If the robot is stationary for a long time, the drawn trail may lag until motion resumes; use `path_odom_stride:=1` or add a separate publish-on-prune mode if you need that.

## Candidate implementations

- AMCL (Nav2 adaptive Monte Carlo localisation)
- RTAB-Map localisation mode

## TODO

- [ ] **Non-EKF localisation path:** Prefer the standard TF split before adding `robot_localisation` or another EKF: publish **`map` → `odom`** from exteroceptive SLAM / localisation (e.g. RTAB-Map with ZED), keep the corrected visual-odometry **`odom` → `base_link`** path, and validate **`map` → `odom` → `base_link`** composition in RViz. Only introduce sensor fusion EKF/UKF if TF + SLAM priors are insufficient for Nav2 or a specific downstream requirement.

## Dependencies (planned)

- `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Localisation framework package
