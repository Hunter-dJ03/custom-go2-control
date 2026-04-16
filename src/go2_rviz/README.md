# go2_rviz

Visualization configuration files for RViz2 and Foxglove.

## Available Configs

| Config | Fixed Frame | Displays |
|--------|-------------|----------|
| `go2_default.rviz` | `map` | RobotModel, TF, PointCloud2 (LiDAR), Image (front camera), Odometry |
| `go2_nav.rviz` | `map` | RobotModel, TF, Map, Path, LaserScan, Odometry |

## Usage

```bash
rviz2 -d $(ros2 pkg prefix go2_rviz)/share/go2_rviz/rviz/go2_default.rviz
```

```bash
rviz2 -d $(ros2 pkg prefix go2_rviz)/share/go2_rviz/rviz/go2_nav.rviz
```

## Foxglove Studio

Phase 1 starts **`foxglove_bridge`** by default so you can visualize without RViz.

1. Install [Foxglove Studio](https://foxglove.dev/download) (desktop or web).
2. Launch the stack: `ros2 launch go2_bringup phase1.launch.py` (bridge listens on **8765**). Topic filtering is defined in **`go2_bringup/config/foxglove_phase1.yaml`** (`topic_whitelist` regex list aligned with `go2_default.rviz`: TF, `robot_description`, `/go2/*`, `/utlidar/*`, Unitree state topics, `rosout`, `parameter_events`).
3. In Studio: **Open connection** → **Foxglove WebSocket** → `ws://127.0.0.1:8765` (or your robot’s IP from another machine).
4. Add panels (3D, Image, Raw Messages, etc.) and subscribe to the same topics as the RViz config, for example:
   - `/robot_description`, `/tf`, `/tf_static`
   - `/utlidar/cloud_deskewed`, `/go2/front_camera/image_raw`, `/go2/odom`, `/go2/path`

Disable the bridge or switch back to RViz with launch args, for example:

```bash
ros2 launch go2_bringup phase1.launch.py foxglove:=false rviz:=true
```

**Topic filtering:** `foxglove_bridge` uses **`topic_whitelist`** (ECMAScript regex strings; a topic is advertised if **any** pattern matches). Defaults live in **`foxglove_phase1.yaml`** so the list is a real YAML array (launch-string lists were unreliable). To advertise **everything** on the DDS graph:

```bash
ros2 launch go2_bringup phase1.launch.py \
  foxglove_params_file:=$(ros2 pkg prefix go2_bringup)/share/go2_bringup/config/foxglove_all_topics.yaml
```

Edit `foxglove_phase1.yaml` to add patterns (e.g. `/uslam/`) or duplicate it for a custom file passed via **`foxglove_params_file`**.

The `foxglove/` directory is reserved for Foxglove Studio **layout** JSON exports if you want to version a default panel layout.
