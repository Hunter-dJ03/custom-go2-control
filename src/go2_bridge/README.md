# go2_bridge

Bidirectional bridge between Unitree DDS topics and standard ROS 2 types.

## Layer

1 — Vendor & Transport

## Purpose

The **only** package in the workspace that depends on `unitree_go` and
`unitree_api` vendor message packages. All other packages use standard
ROS 2 types and `go2_interfaces`. This isolation means upstream Unitree
changes never propagate beyond the bridge.

Performs:
- Type conversion (Unitree types → standard ROS 2 types)
- TF broadcasting (odom → base_link)
- H264 front camera decoding (NVDEC hardware accelerated when available)
- Generic sport API service (fire-and-forget and request-response patterns)

Contains **zero business logic** — pure transport and data conversion.

## Nodes

### go2_bridge_node

**Published topics:**

| Topic | Type | Source |
|-------|------|--------|
| `/go2/odom` | nav_msgs/Odometry | `/sportmodestate` |
| `/go2/imu` | sensor_msgs/Imu | `/lowstate` |
| `/go2/joint_states` | sensor_msgs/JointState | `/lowstate` motor_state[0..11] |
| `/go2/battery` | sensor_msgs/BatteryState | `/lowstate` BMS (throttled) |

### joint_states_throttle_node

Relays the latest **`/go2/joint_states`** at a fixed rate (default **20 Hz**) on **`/go2/lf/joint_states`** for remote visualization. Onboard **`robot_state_publisher`** keeps subscribing to full-rate `/go2/joint_states`.

**Parameters:** `input_topic`, `output_topic`, `publish_rate_hz` (see `config/joint_states_throttle.yaml`).

### tf_lf_relay_node

Buffers dynamic transforms from **`/tf`**, drops leg link frames (default: child frames matching `^(FR|FL|RR|RL)_`), and republishes the rest at a fixed rate (default **20 Hz**) on **`/go2/lf/tf`**. Use with **`/go2/lf/joint_states`** in Foxglove instead of full **`/tf`**.

**Parameters:** `input_tf_topic`, `output_tf_topic`, `publish_rate_hz`, `exclude_child_frame_patterns` (see `config/tf_lf_relay.yaml`).

`header.stamp` on `/go2/odom`, `/go2/imu`, and joints/battery paths uses the node's ROS time (`now()`), not embedded Unitree MCU stamps — avoids TF / perception timestamp skew when compared to calibrated LiDAR (which may optionally restamp in `go2_lidar`).

Planned (not implemented in this node yet): foot_force, motion raw_state, lidar relay, camera decode, joy.

**TF:** `odom` → `base_link`

**Services:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/sport_api_call` | go2_interfaces/SportApiCall | Publish to `/api/sport/request` (fire-and-forget) |

**Parameters:** `odom_frame`, `base_frame`, `sport_mode_state_topic`, `low_state_topic`

## Safety

**Critical** — sole gateway to robot hardware. Loss of this node means
complete loss of robot communication.

## Dependencies

- `unitree_go` (vendor)
- `unitree_api` (vendor)
- `go2_interfaces`
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`
