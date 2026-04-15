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
