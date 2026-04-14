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

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/go2/odom` | nav_msgs/Odometry | 50 Hz | sportmodestate |
| `/go2/imu/data` | sensor_msgs/Imu | 50 Hz | sportmodestate/lowstate IMU |
| `/go2/joint_states` | sensor_msgs/JointState | 50 Hz | lowstate motor_state |
| `/go2/foot_force` | go2_interfaces/FootForce | 50 Hz | lowstate foot_force |
| `/go2/battery` | sensor_msgs/BatteryState | 2 Hz | lf/lowstate |
| `/go2/motion/raw_state` | go2_interfaces/MotionState | 50 Hz | sportmodestate |
| `/go2/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz | utlidar/cloud |
| `/go2/front_camera/image_raw` | sensor_msgs/Image | 30 Hz | frontvideostream |
| `/go2/joy` | sensor_msgs/Joy | event | /wirelesscontroller |

**TF:** odom → base_link

**Services:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/bridge/sport_api_call` | SportApiCall | Send sport API commands to robot |
| `/go2/bridge/robot_state_call` | SportApiCall | Query robot state services |

## Safety

**Critical** — sole gateway to robot hardware. Loss of this node means
complete loss of robot communication.

## Dependencies

- `unitree_go` (vendor)
- `unitree_api` (vendor)
- `go2_interfaces`
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`
