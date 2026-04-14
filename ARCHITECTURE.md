# Go2 ROS 2 System Architecture

Unitree Go2 Pro — AGX Orin Onboard Compute — ROS 2 Humble

---

## 1. System Context

```
┌─────────────────────────────────────────────────────────┐
│                   Go2 Internal MCU                      │
│                                                         │
│  Firmware ~v1.1.11 (secondary development enabled)      │
│  CycloneDDS v0.10.2                                     │
│  Sport mode controller, IMU, motor drivers, LiDAR,      │
│  front camera (H264), battery management                │
│                                                         │
│  Publishes/subscribes DDS topics on 192.168.123.x       │
└──────────────────────┬──────────────────────────────────┘
                       │ Ethernet (enp2s0)
                       │ CycloneDDS multicast
┌──────────────────────▼──────────────────────────────────┐
│              NVIDIA Jetson AGX Orin (onboard)            │
│                                                         │
│  ROS 2 Humble + rmw_cyclonedds_cpp                      │
│  Primary compute: perception, planning, control          │
│  ZED camera, additional sensors                          │
│  This architecture runs here                             │
└──────────────────────┬──────────────────────────────────┘
                       │ WiFi (wlp3s0f0)
┌──────────────────────▼──────────────────────────────────┐
│              Base Station (laptop)                       │
│                                                         │
│  RViz2, Foxglove, teleoperation                          │
│  Monitoring and debugging only                           │
│  No safety-critical functions                            │
└─────────────────────────────────────────────────────────┘
```

### DDS Topic Landscape (published by robot firmware)

The Go2 firmware publishes directly to CycloneDDS. These appear as native
ROS 2 topics when `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set.

| DDS Topic | Message Type | Rate | Direction |
|-----------|-------------|------|-----------|
| `sportmodestate` | `unitree_go/SportModeState` | ~50 Hz | Robot → Orin |
| `lf/sportmodestate` | `unitree_go/SportModeState` | ~2 Hz | Robot → Orin |
| `lowstate` | `unitree_go/LowState` | ~500 Hz | Robot → Orin |
| `lf/lowstate` | `unitree_go/LowState` | ~2 Hz | Robot → Orin |
| `utlidar/cloud` | `sensor_msgs/PointCloud2` | ~10 Hz | Robot → Orin |
| `utlidar/range_info` | `geometry_msgs/PointStamped` | ~10 Hz | Robot → Orin |
| `utlidar/height_map_array` | `unitree_go/HeightMap` | ~10 Hz | Robot → Orin |
| `frontvideostream` | `unitree_go/Go2FrontVideoData` | ~30 Hz | Robot → Orin |
| `/wirelesscontroller` | `unitree_go/WirelessController` | event | Robot → Orin |
| `/api/sport/request` | `unitree_api/Request` | on demand | Orin → Robot |
| `/api/sport/response` | `unitree_api/Response` | on demand | Robot → Orin |
| `/api/robot_state/request` | `unitree_api/Request` | on demand | Orin → Robot |
| `/api/robot_state/response` | `unitree_api/Response` | on demand | Robot → Orin |

---

## 2. Architecture Layers

```
┌─────────────────────────────────────────────────────────┐
│  7. OPERATOR & TOOLING                                   │
│     go2_teleop · go2_basestation · go2_bringup           │
├─────────────────────────────────────────────────────────┤
│  6. DESCRIPTION & SIMULATION                             │
│     go2_description · go2_sim                            │
├─────────────────────────────────────────────────────────┤
│  5. AUTONOMY                                             │
│     go2_perception · go2_localization · go2_navigation   │
├─────────────────────────────────────────────────────────┤
│  4. SENSORS                                              │
│     go2_lidar · go2_camera                               │
├─────────────────────────────────────────────────────────┤
│  3. ROBOT STATE & HEALTH                                 │
│     go2_state                                            │
├─────────────────────────────────────────────────────────┤
│  2. ROBOT CONTROL ABSTRACTION                            │
│     go2_motion · go2_arbitration                         │
├─────────────────────────────────────────────────────────┤
│  1. VENDOR & TRANSPORT                                   │
│     go2_bridge · go2_interfaces                          │
│     unitree_go (vendor) · unitree_api (vendor)           │
└─────────────────────────────────────────────────────────┘
```

---

## 3. Workspace Structure

```
~/unitree_ws/
├── ARCHITECTURE.md              ← this document
├── setup_env.bash               ← unified environment script
├── src/
│   ├── vendor/                  ← third-party, do not modify
│   │   ├── unitree_go/          ← Unitree Go2 message definitions
│   │   └── unitree_api/         ← Unitree API message definitions
│   │
│   ├── go2_interfaces/          ← custom msgs/srvs/actions
│   ├── go2_bridge/              ← Unitree DDS ↔ standard ROS 2
│   ├── go2_motion/              ← unified motion abstraction
│   ├── go2_arbitration/         ← command mux + safety
│   ├── go2_state/               ← state aggregation + health
│   ├── go2_lidar/               ← LiDAR processing
│   ├── go2_camera/              ← front camera + ZED integration
│   ├── go2_perception/          ← detection, classification
│   ├── go2_localization/        ← SLAM + localization
│   ├── go2_navigation/          ← Nav2 + planning
│   ├── go2_teleop/              ← teleoperation
│   ├── go2_description/         ← URDF/Xacro + meshes
│   ├── go2_bringup/             ← onboard launch orchestration
│   ├── go2_basestation/         ← base station launch + viz
│   └── go2_sim/                 ← simulation support
│
├── build/
├── install/
└── log/
```

### Vendor Isolation

`unitree_go` and `unitree_api` are the only packages that define Unitree-specific
message types. **Only `go2_bridge` depends on these vendor packages.** All other
packages depend on `go2_interfaces` and standard ROS 2 message types. This means:

- Vendor packages can be updated independently
- Upstream Unitree changes do not propagate beyond the bridge
- The rest of the system is portable to other robots

---

## 4. Package Specifications

### 4.1 `unitree_go` (VENDOR — read-only)

**Source:** `unitree_ros2/cyclonedds_ws/src/unitree/unitree_go`
**Type:** `ament_cmake` (rosidl interface package)
**Owner:** Unitree Robotics

24 message types including `SportModeState`, `LowState`, `LowCmd`,
`Go2FrontVideoData`, `IMUState`, `MotorState`, `BmsState`,
`WirelessController`, `HeightMap`, `LidarState`, `PathPoint`.

### 4.2 `unitree_api` (VENDOR — read-only)

**Source:** `unitree_ros2/cyclonedds_ws/src/unitree/unitree_api`
**Type:** `ament_cmake` (rosidl interface package)
**Owner:** Unitree Robotics

8 message types: `Request`, `Response`, `RequestHeader`, `RequestIdentity`,
`RequestLease`, `RequestPolicy`, `ResponseHeader`, `ResponseStatus`.

These implement the JSON-over-DDS RPC pattern used by all Unitree service APIs.

---

### 4.3 `go2_interfaces`

**Type:** `ament_cmake` (rosidl interface package)
**Owner:** You
**Layer:** Cross-cutting (used by all layers)
**Safety:** Non-critical (type definitions only)

The single source of truth for all custom message, service, and action
definitions in the system. No nodes. No logic.

#### Messages

| Message | Purpose |
|---------|---------|
| `MotionMode.msg` | Enum-like: current motion mode (idle, standing, walking, sitting, executing_action, damp, fault) |
| `MotionState.msg` | Normalized motion state: mode, gait_type, body_height, foot_raise_height, capabilities_available, progress |
| `ArbitrationStatus.msg` | Active source, priority level, safety state, timestamp of last command |
| `RobotHealth.msg` | Battery SOC, voltage, current, motor temperatures[12], IMU temperature, fault codes, uptime |
| `FootForce.msg` | int16[4] raw force + int16[4] estimated force |

#### Services

| Service | Purpose |
|---------|---------|
| `SetGait.srv` | Request: uint8 gait_type → Response: bool success, string message |
| `SetPosture.srv` | Request: uint8 posture (STAND, SIT, DAMP, BALANCE_STAND, RECOVERY) → Response: bool success |
| `SetBodyHeight.srv` | Request: float32 height → Response: bool success |
| `SetFootRaiseHeight.srv` | Request: float32 height → Response: bool success |
| `GetCapabilities.srv` | Request: (empty) → Response: string[] available_motions, string[] unavailable_motions |
| `SetArbitrationSource.srv` | Request: uint8 source (TELEOP, AUTONOMY, SCRIPT, NONE) → Response: bool success |
| `EStop.srv` | Request: bool engage → Response: bool success |
| `SportApiCall.srv` | Request: int32 api_id, string parameter_json → Response: bool success, string response_json |

#### Actions

| Action | Purpose |
|--------|---------|
| `ExecuteMotion.action` | Goal: uint8 motion_id, string params_json / Feedback: float32 progress, string state / Result: bool success, string message |

---

### 4.4 `go2_bridge`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 1 — Vendor & Transport
**Runs on:** Onboard only
**Safety:** Critical (sole gateway to robot hardware)

The only package that depends on `unitree_go` and `unitree_api`. It performs
bidirectional type conversion between Unitree DDS messages and standard ROS 2
types. Contains zero business logic — pure transport and data conversion.

#### Nodes

**`go2_bridge_node`** (single composable node, or split into sub-nodes)

**Subscriptions (from robot DDS):**

| Subscribes to | Type | QoS |
|---------------|------|-----|
| `sportmodestate` | `unitree_go/SportModeState` | Best effort, 1 |
| `lowstate` | `unitree_go/LowState` | Best effort, 1 |
| `lf/lowstate` | `unitree_go/LowState` | Best effort, 1 |
| `utlidar/cloud` | `sensor_msgs/PointCloud2` | Best effort, 5 |
| `frontvideostream` | `unitree_go/Go2FrontVideoData` | Best effort, 1 |
| `/wirelesscontroller` | `unitree_go/WirelessController` | Best effort, 1 |
| `/api/sport/response` | `unitree_api/Response` | Reliable, 10 |
| `/api/robot_state/response` | `unitree_api/Response` | Reliable, 10 |

**Publications (standard ROS 2):**

| Publishes to | Type | Rate | Source |
|-------------|------|------|--------|
| `/go2/odom` | `nav_msgs/Odometry` | 50 Hz | `sportmodestate` position + velocity |
| `/go2/imu/data` | `sensor_msgs/Imu` | 50 Hz | `sportmodestate` or `lowstate` IMU |
| `/go2/joint_states` | `sensor_msgs/JointState` | 50 Hz | `lowstate` motor_state[0..11] |
| `/go2/foot_force` | `go2_interfaces/FootForce` | 50 Hz | `lowstate` foot_force |
| `/go2/battery` | `sensor_msgs/BatteryState` | 2 Hz | `lf/lowstate` bms_state + power |
| `/go2/motion/raw_state` | `go2_interfaces/MotionState` | 50 Hz | `sportmodestate` mode/gait/progress |
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz | `utlidar/cloud` (pass-through, reframe) |
| `/go2/front_camera/image_raw` | `sensor_msgs/Image` | 30 Hz | `frontvideostream` (H264 decode) |
| `/go2/joy` | `sensor_msgs/Joy` | event | `/wirelesscontroller` mapped to Joy axes/buttons |

**TF broadcasts:**

| Parent | Child | Source |
|--------|-------|--------|
| `odom` | `base_link` | `sportmodestate` position + IMU quaternion |

**Services provided:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/bridge/sport_api_call` | `go2_interfaces/SportApiCall` | Sends `unitree_api/Request` to `/api/sport/request`, waits for matching response |
| `/go2/bridge/robot_state_call` | `go2_interfaces/SportApiCall` | Same pattern for `/api/robot_state/request` |

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `network_interface` | `enp2s0` | CycloneDDS interface |
| `publish_tf` | `true` | Enable odom→base_link TF |
| `decode_front_camera` | `true` | Enable H264 decoding (CPU intensive) |
| `joint_names` | `[FL_hip, FL_thigh, ...]` | 12 joint names for JointState |

#### Design notes

- H264 decoding uses hardware-accelerated NVDEC on the Orin when available,
  falling back to FFmpeg software decode.
- The `sportmodestate` subscription at 50 Hz is the primary source for odom
  and IMU. The `lowstate` at 500 Hz is available but typically downsampled.
  Use `lf/lowstate` (~2 Hz) for battery/diagnostics to avoid bandwidth waste.
- The bridge does NOT contain any `SportClient` logic (JSON construction,
  API ID constants). It provides a generic `sport_api_call` service.
  The `go2_motion` package owns the semantic API mapping.

---

### 4.5 `go2_motion`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 2 — Robot Control Abstraction
**Runs on:** Onboard only
**Safety:** Critical (commands reach the robot through this)

Unified high-level motion interface built on the Sport API. Owns the
`SportClient` equivalent logic: API ID constants, JSON parameter encoding,
motion state machine, capability abstraction.

#### Nodes

**`go2_motion_node`**

**Subscriptions:**

| Subscribes to | Type | Purpose |
|---------------|------|---------|
| `/go2/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (from arbitration) |
| `/go2/motion/raw_state` | `go2_interfaces/MotionState` | Track current robot mode |

**Publications:**

| Publishes to | Type | Rate | Purpose |
|-------------|------|------|---------|
| `/go2/motion/state` | `go2_interfaces/MotionState` | 10 Hz | Processed state with capability info |

**Services provided:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/motion/set_gait` | `go2_interfaces/SetGait` | Switch gait (trot, run, climb, static, economic, etc.) |
| `/go2/motion/set_posture` | `go2_interfaces/SetPosture` | Posture transition (stand, sit, damp, balance, recovery) |
| `/go2/motion/set_body_height` | `go2_interfaces/SetBodyHeight` | Adjust body height |
| `/go2/motion/set_foot_raise_height` | `go2_interfaces/SetFootRaiseHeight` | Adjust step height |
| `/go2/motion/get_capabilities` | `go2_interfaces/GetCapabilities` | Query available motions for current firmware |

**Action servers:**

| Action | Type | Purpose |
|--------|------|---------|
| `/go2/motion/execute` | `go2_interfaces/ExecuteMotion` | Triggered motions (hello, stretch, flip, dance, sit, stand) |

**Service clients (calls go2_bridge):**

| Client | Type |
|--------|------|
| `/go2/bridge/sport_api_call` | `go2_interfaces/SportApiCall` |

#### Internal design

```
                      ┌─────────────────────────┐
  /go2/cmd_vel ──────►│                         │
                      │    Motion State Machine  │──► /go2/bridge/sport_api_call
  /go2/motion/        │                         │
   set_gait ─────────►│  States:                │
   set_posture ──────►│   IDLE                  │
   execute (action) ──►│   STANDING              │
                      │   WALKING               │
  /go2/motion/        │   SITTING               │
   raw_state ────────►│   EXECUTING_ACTION      │
                      │   DAMP                  │
                      │   FAULT                 │
                      │                         │──► /go2/motion/state
                      └─────────────────────────┘
```

- `cmd_vel` is converted to `SportClient::Move(vx, vy, vyaw)` at 10-20 Hz
- Posture transitions are sequenced (e.g., must stand before walking)
- Special motions (flips, dances) set `EXECUTING_ACTION` state and reject
  velocity commands until complete
- Capability abstraction: queries firmware version via `RobotStateClient`,
  maintains a set of available `api_id` values. Returns clear errors for
  unsupported motions rather than silent failures.

---

### 4.6 `go2_arbitration`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 2 — Robot Control Abstraction
**Runs on:** Onboard only
**Safety:** **CRITICAL** (the single gatekeeper for all robot commands)

All command sources must pass through arbitration before reaching the robot.
Implements priority-based command muxing, safety gating, watchdog timeout,
and emergency stop.

Modeled after the `joint_desired_control_mux` pattern from the rover
architecture, extended with priority levels and safety monitoring.

#### Nodes

**`go2_arbitration_node`**

**Subscriptions (command inputs):**

| Subscribes to | Type | Source | Priority |
|---------------|------|--------|----------|
| `/go2/teleop/cmd_vel` | `geometry_msgs/Twist` | go2_teleop | 2 (medium) |
| `/go2/nav/cmd_vel` | `geometry_msgs/Twist` | go2_navigation | 1 (low) |
| `/go2/script/cmd_vel` | `geometry_msgs/Twist` | scripted behaviors | 1 (low) |
| `/go2/teleop/posture` | `go2_interfaces/SetPosture` | teleop posture cmds | 2 (medium) |
| `/go2/motion/state` | `go2_interfaces/MotionState` | motion state feedback | — |

**Publications:**

| Publishes to | Type | Rate | Purpose |
|-------------|------|------|---------|
| `/go2/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Arbitrated velocity to go2_motion |
| `/go2/arbitration/status` | `go2_interfaces/ArbitrationStatus` | 5 Hz | Active source, safety state |

**Services provided:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/arbitration/set_source` | `go2_interfaces/SetArbitrationSource` | Switch active command source |
| `/go2/arbitration/estop` | `go2_interfaces/EStop` | Emergency stop — sends Damp immediately |

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `cmd_timeout_ms` | `500` | Watchdog: zero velocity if no command received |
| `estop_on_disconnect` | `true` | Damp robot if teleop heartbeat lost |
| `default_source` | `TELEOP` | Active source on startup |

#### Priority and safety rules

1. **E-Stop** (priority 255): always wins, sends `Damp` (API 1001) immediately
2. **Teleop** (priority 2): operator override, preempts autonomy
3. **Autonomy / Script** (priority 1): normal operation
4. **Watchdog**: if the active source stops publishing for `cmd_timeout_ms`,
   publishes zero velocity. If timeout exceeds 3× threshold, triggers `StopMove`.
5. **Fault response**: if `go2_motion/state` reports FAULT, rejects all
   velocity commands and sends `RecoveryStand` (API 1006) once.

---

### 4.7 `go2_state`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 3 — Robot State & Health
**Runs on:** Onboard only
**Safety:** Important (monitoring, non-blocking)

Aggregates robot health from multiple sources. Separates high-rate
control-relevant data (handled by `go2_bridge`) from low-rate diagnostics.
Monitors for faults and publishes diagnostic summaries.

#### Nodes

**`go2_state_node`**

**Subscriptions:**

| Subscribes to | Type | Purpose |
|---------------|------|---------|
| `/go2/battery` | `sensor_msgs/BatteryState` | Battery monitoring |
| `/go2/joint_states` | `sensor_msgs/JointState` | Motor temperature monitoring (via effort field or separate topic) |
| `/go2/motion/state` | `go2_interfaces/MotionState` | Error code tracking |
| `/go2/odom` | `nav_msgs/Odometry` | Liveness check |

**Publications:**

| Publishes to | Type | Rate | Purpose |
|-------------|------|------|---------|
| `/go2/health` | `go2_interfaces/RobotHealth` | 1 Hz | Aggregated health summary |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | ROS 2 standard diagnostics |

**Service clients:**

| Client | Type | Purpose |
|--------|------|---------|
| `/go2/bridge/robot_state_call` | `go2_interfaces/SportApiCall` | Query service list, report frequency |

#### Fault monitoring

- Battery SOC < 15%: publish warning
- Battery SOC < 5%: trigger sit-down via arbitration e-stop
- Motor temperature > 70°C: publish warning
- Motor temperature > 85°C: trigger damp
- No odom received for > 2s: connection loss alarm
- `SportModeState.error_code != 0`: parse and publish fault description

---

### 4.8 `go2_lidar`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 4 — Sensors
**Runs on:** Onboard only
**Safety:** Non-critical

Processes raw LiDAR data from the Unitree L2 LiDAR into usable formats
for downstream SLAM and navigation.

#### Nodes

**`go2_lidar_node`**

**Subscriptions:**

| Subscribes to | Type |
|---------------|------|
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` |

**Publications:**

| Publishes to | Type | Purpose |
|-------------|------|---------|
| `/go2/lidar/scan` | `sensor_msgs/LaserScan` | 2D scan slice for Nav2 costmap |
| `/go2/lidar/points_filtered` | `sensor_msgs/PointCloud2` | Filtered/downsampled cloud for SLAM |

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `scan_height_min` | `-0.1` | Slice height for 2D scan extraction |
| `scan_height_max` | `0.3` | Slice height for 2D scan extraction |
| `voxel_size` | `0.05` | Voxel grid filter resolution |
| `range_min` | `0.15` | Minimum range filter |
| `range_max` | `30.0` | Maximum range filter |

---

### 4.9 `go2_camera`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 4 — Sensors
**Runs on:** Onboard only
**Safety:** Non-critical

Manages camera data ingestion. The built-in front camera is decoded by
`go2_bridge`. This package handles:
- Camera info and calibration publishing for the front camera
- ZED camera integration (via `zed_ros2_wrapper`)
- Time synchronization between camera sources

#### Nodes

**`go2_front_camera_info_node`** (lightweight)

| Publishes to | Type | Purpose |
|-------------|------|---------|
| `/go2/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration for front camera |

**Parameters:**
- `calibration_file`: path to front camera calibration YAML

**ZED integration**: launched separately via `zed_ros2_wrapper` in bringup.
This package provides launch files and config that integrate ZED topics
into the `/go2/` namespace via remapping.

**TF:** Static transforms from `base_link` to camera frames are published
by `go2_description` via `robot_state_publisher`, not by this package.

---

### 4.10 `go2_perception`

**Type:** `ament_cmake` (C++ / Python hybrid)
**Owner:** You
**Layer:** 5 — Autonomy
**Runs on:** Onboard only
**Safety:** Non-critical
**Status:** DEFERRED — implement after core stack is validated

Object detection, terrain classification, and other perception tasks.
Consumes camera and LiDAR data, publishes detections and classifications.

---

### 4.11 `go2_localization`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 5 — Autonomy
**Runs on:** Onboard only
**Safety:** Important (autonomy depends on this)
**Status:** DEFERRED — implement after sensor pipeline is validated

Custom SLAM and localization. Not using Unitree's built-in SLAM
(requires expansion dock hardware).

**Subscriptions:** `/go2/lidar/points_filtered`, `/go2/odom`, `/go2/imu/data`

**Publications:** `/go2/map` (nav_msgs/OccupancyGrid), TF `map → odom`

Candidate implementations: RTAB-Map, Point-LIO, LIO-SAM, or similar.

---

### 4.12 `go2_navigation`

**Type:** `ament_cmake` (C++ / Python)
**Owner:** You
**Layer:** 5 — Autonomy
**Runs on:** Onboard only
**Safety:** Important
**Status:** DEFERRED — implement after localization is working

Nav2 integration. Outputs velocity commands to `/go2/nav/cmd_vel`
which go through arbitration before reaching the motion layer.

**Publications:**

| Publishes to | Type | Purpose |
|-------------|------|---------|
| `/go2/nav/cmd_vel` | `geometry_msgs/Twist` | Planned velocity to arbitration |

**Subscriptions:** `/go2/map`, TF, `/go2/lidar/scan`, costmap sources

---

### 4.13 `go2_teleop`

**Type:** `ament_cmake` (C++)
**Owner:** You
**Layer:** 7 — Operator & Tooling
**Runs on:** Base station (or onboard for wireless controller)
**Safety:** Important (operator control path)

Teleoperation interfaces. Two input paths:

1. **Wireless controller** (onboard): Subscribes to `/go2/joy` from
   `go2_bridge`, maps to velocity commands
2. **Joystick/keyboard** (base station): Standard `joy_node` → twist mapping

#### Nodes

**`go2_teleop_node`**

**Subscriptions:**

| Subscribes to | Type | Source |
|---------------|------|--------|
| `/go2/joy` | `sensor_msgs/Joy` | Unitree wireless controller (via bridge) |
| `/joy` | `sensor_msgs/Joy` | External joystick (base station) |

**Publications:**

| Publishes to | Type |
|-------------|------|
| `/go2/teleop/cmd_vel` | `geometry_msgs/Twist` |

**Services called:**

| Service | Purpose |
|---------|---------|
| `/go2/motion/set_posture` | Button-mapped posture changes |
| `/go2/motion/execute` | Button-mapped special motions |
| `/go2/arbitration/estop` | Emergency stop button |

---

### 4.14 `go2_description`

**Type:** `ament_cmake`
**Owner:** You
**Layer:** 6 — Description & Simulation
**Runs on:** Both (onboard for TF, base station for viz)
**Safety:** Non-critical

Robot model in URDF/Xacro. Following the `kanga_description` pattern:
two-layer xacro with macros and assembly files.

```
go2_description/
├── urdf/
│   ├── go2_macro.urdf.xacro      ← base body + legs (visual only, no transmission)
│   ├── go2.urdf.xacro            ← assembly: body + sensors
│   ├── sensors/
│   │   ├── lidar.urdf.xacro      ← Unitree L2 LiDAR mount
│   │   ├── front_camera.urdf.xacro
│   │   └── zed.urdf.xacro        ← ZED camera mount
│   └── payloads/
│       └── orin_payload.urdf.xacro
├── meshes/
│   ├── base_link.dae
│   └── ...
├── config/
│   └── joint_names.yaml
├── launch/
│   └── rsp.launch.py             ← robot_state_publisher
└── rviz/
    └── go2.rviz
```

**Nodes launched:** `robot_state_publisher` (publishes static TF tree)

**Key frames:** `base_link`, `imu_link`, `lidar_link`, `front_camera_link`,
`zed_camera_link`, `FL_hip`, `FL_thigh`, `FL_calf`, etc.

Note: joint states for legs come from `/go2/joint_states` (via go2_bridge).
The URDF does not define transmissions since we do not do low-level motor control.

---

### 4.15 `go2_bringup`

**Type:** `ament_cmake`
**Owner:** You
**Layer:** 7 — Operator & Tooling
**Runs on:** Onboard only
**Safety:** Non-critical (launch orchestration only)

No nodes. Only launch files, config, and parameter files.
Following the `kanga_bringup` pattern: thin orchestration layer.

```
go2_bringup/
├── launch/
│   ├── robot.launch.py           ← full onboard stack
│   ├── bridge.launch.py          ← bridge + state only (testing)
│   ├── navigation.launch.py      ← SLAM + Nav2
│   └── sensors.launch.py         ← LiDAR + cameras
├── config/
│   ├── bridge_params.yaml
│   ├── motion_params.yaml
│   ├── arbitration_params.yaml
│   ├── nav2_params.yaml
│   └── cyclonedds.xml
└── README.md
```

**Dependencies:** all `go2_*` packages (exec_depend only)

---

### 4.16 `go2_basestation`

**Type:** `ament_cmake`
**Owner:** You
**Layer:** 7 — Operator & Tooling
**Runs on:** Base station only
**Safety:** Non-critical

Launch files for the base station computer. Includes RViz config,
Foxglove Bridge, teleop, and rosbag recording.

```
go2_basestation/
├── launch/
│   ├── basestation.launch.py     ← RViz + teleop + Foxglove
│   ├── record.launch.py          ← rosbag recording
│   └── playback.launch.py        ← rosbag playback + viz
├── config/
│   ├── foxglove_bridge.yaml
│   └── go2.rviz
└── README.md
```

**Dependencies:** `go2_description`, `go2_interfaces`, `go2_teleop`,
`foxglove_bridge`, `rosbag2_transport`

---

### 4.17 `go2_sim`

**Type:** `ament_cmake`
**Owner:** You
**Layer:** 6 — Description & Simulation
**Runs on:** Development machine
**Safety:** Non-critical
**Status:** DEFERRED

Simulation support (Gazebo, Isaac Sim, or MuJoCo). Key requirement:
the same `go2_motion` and `go2_arbitration` interfaces work in sim
and real deployment — the bridge layer is swapped for a sim bridge.

---

## 5. Data Flow Diagram

```
                    ┌──────────────┐
                    │  Robot MCU   │
                    │  (firmware)  │
                    └──────┬───────┘
                           │ DDS (Ethernet)
                    ┌──────▼───────┐
                    │  go2_bridge  │
                    └──┬───┬───┬───┘
         ┌─────────────┤   │   ├─────────────────┐
         ▼             ▼   │   ▼                  ▼
    /go2/odom    /go2/imu  │  /go2/lidar    /go2/front_camera
    /go2/joint_states      │  /points       /image_raw
    /go2/battery           │
    /go2/motion/raw_state  │  ┌──────────┐
         │                 │  │go2_lidar │
         ▼                 │  └────┬─────┘
    ┌──────────┐           │       ▼
    │go2_state │           │  /go2/lidar/scan
    └────┬─────┘           │  /go2/lidar/points_filtered
         ▼                 │       │
    /go2/health            │       ▼
    /diagnostics           │  ┌────────────────┐
                           │  │go2_localization│
                           │  └────────┬───────┘
                           │           ▼
                           │     TF: map→odom
                           │     /go2/map
                           │           │
                           │           ▼
                           │  ┌────────────────┐
                           │  │go2_navigation  │
                           │  └────────┬───────┘
                           │           │
                           │     /go2/nav/cmd_vel
                           │           │
  /go2/teleop/cmd_vel      │           │
       │                   │           │
       ▼                   │           ▼
  ┌──────────────────┐     │  ┌────────────────┐
  │   go2_teleop     │     │  │                │
  └────────┬─────────┘     │  │                │
           │               │  │  (all sources) │
           ▼               │  │                │
  ┌────────────────────┐   │  │                │
  │  go2_arbitration   │◄──┘──┘                │
  │  (mux + safety)    │                       │
  └────────┬───────────┘                       │
           │                                   │
     /go2/cmd_vel                              │
           │                                   │
           ▼                                   │
  ┌────────────────┐                           │
  │  go2_motion    │                           │
  │  (state machine│                           │
  │   + API calls) │                           │
  └────────┬───────┘                           │
           │                                   │
  /go2/bridge/sport_api_call                   │
           │                                   │
           ▼                                   │
  ┌────────────────┐                           │
  │  go2_bridge    │───────────────────────────┘
  │  (DDS tx)      │
  └────────┬───────┘
           │ /api/sport/request
           ▼
  ┌────────────────┐
  │  Robot MCU     │
  └────────────────┘
```

---

## 6. Topic, Service, and Action Summary

### Topics

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|----------|---------------|------|
| `/go2/odom` | `nav_msgs/Odometry` | go2_bridge | go2_localization, go2_state | 50 Hz |
| `/go2/imu/data` | `sensor_msgs/Imu` | go2_bridge | go2_localization | 50 Hz |
| `/go2/joint_states` | `sensor_msgs/JointState` | go2_bridge | go2_description (RSP), go2_state | 50 Hz |
| `/go2/foot_force` | `go2_interfaces/FootForce` | go2_bridge | go2_state | 50 Hz |
| `/go2/battery` | `sensor_msgs/BatteryState` | go2_bridge | go2_state | 2 Hz |
| `/go2/motion/raw_state` | `go2_interfaces/MotionState` | go2_bridge | go2_motion | 50 Hz |
| `/go2/motion/state` | `go2_interfaces/MotionState` | go2_motion | go2_arbitration, go2_state, go2_teleop | 10 Hz |
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | go2_bridge | go2_lidar | 10 Hz |
| `/go2/lidar/scan` | `sensor_msgs/LaserScan` | go2_lidar | go2_navigation | 10 Hz |
| `/go2/lidar/points_filtered` | `sensor_msgs/PointCloud2` | go2_lidar | go2_localization | 10 Hz |
| `/go2/front_camera/image_raw` | `sensor_msgs/Image` | go2_bridge | go2_perception | 30 Hz |
| `/go2/front_camera/camera_info` | `sensor_msgs/CameraInfo` | go2_camera | go2_perception | 30 Hz |
| `/go2/joy` | `sensor_msgs/Joy` | go2_bridge | go2_teleop | event |
| `/go2/health` | `go2_interfaces/RobotHealth` | go2_state | go2_basestation | 1 Hz |
| `/go2/teleop/cmd_vel` | `geometry_msgs/Twist` | go2_teleop | go2_arbitration | 10 Hz |
| `/go2/nav/cmd_vel` | `geometry_msgs/Twist` | go2_navigation | go2_arbitration | 10 Hz |
| `/go2/cmd_vel` | `geometry_msgs/Twist` | go2_arbitration | go2_motion | 10 Hz |
| `/go2/arbitration/status` | `go2_interfaces/ArbitrationStatus` | go2_arbitration | go2_basestation | 5 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | go2_state | standard tools | 1 Hz |

### Services

| Service | Type | Server | Client(s) |
|---------|------|--------|-----------|
| `/go2/bridge/sport_api_call` | `SportApiCall` | go2_bridge | go2_motion |
| `/go2/bridge/robot_state_call` | `SportApiCall` | go2_bridge | go2_state |
| `/go2/motion/set_gait` | `SetGait` | go2_motion | go2_teleop, go2_navigation |
| `/go2/motion/set_posture` | `SetPosture` | go2_motion | go2_teleop, go2_arbitration |
| `/go2/motion/set_body_height` | `SetBodyHeight` | go2_motion | go2_teleop |
| `/go2/motion/set_foot_raise_height` | `SetFootRaiseHeight` | go2_motion | go2_teleop |
| `/go2/motion/get_capabilities` | `GetCapabilities` | go2_motion | go2_teleop |
| `/go2/arbitration/set_source` | `SetArbitrationSource` | go2_arbitration | go2_teleop, go2_basestation |
| `/go2/arbitration/estop` | `EStop` | go2_arbitration | go2_teleop, go2_state, go2_basestation |

### Actions

| Action | Type | Server | Client(s) |
|--------|------|--------|-----------|
| `/go2/motion/execute` | `ExecuteMotion` | go2_motion | go2_teleop, scripted behaviors |

### TF Tree

```
map ──► odom ──► base_link ──┬──► imu_link
    (localization) (bridge)  ├──► lidar_link
                             ├──► front_camera_link
                             ├──► zed_camera_link
                             ├──► FL_hip ──► FL_thigh ──► FL_calf ──► FL_foot
                             ├──► FR_hip ──► FR_thigh ──► FR_calf ──► FR_foot
                             ├──► RL_hip ──► RL_thigh ──► RL_calf ──► RL_foot
                             └──► RR_hip ──► RR_thigh ──► RR_calf ──► RR_foot
```

---

## 7. Vendor vs Owned

| Component | Status | Rationale |
|-----------|--------|-----------|
| `unitree_go` | **Vendor** (read-only) | Unitree message definitions, must match firmware |
| `unitree_api` | **Vendor** (read-only) | Unitree API message definitions |
| `unitree_ros2` examples | **Reference only** | Not built as part of workspace; copy patterns into owned code |
| `go2_ros2_sdk` (community) | **Deprecated** | Phase out; WebRTC path replaced by direct DDS |
| `zed_ros2_wrapper` | **Vendor** (apt/source) | ZED SDK ROS 2 wrapper, used as-is |
| `nav2` | **Vendor** (apt) | Navigation framework, configured via params |
| All `go2_*` packages | **Owned** | Your code, your responsibility |

---

## 8. Deployment Map

### Onboard (AGX Orin)

| Package | Priority |
|---------|----------|
| go2_bridge | **Required** |
| go2_motion | **Required** |
| go2_arbitration | **Required** |
| go2_state | **Required** |
| go2_description (RSP) | **Required** |
| go2_lidar | Required for nav |
| go2_camera | Required for perception |
| go2_localization | Required for autonomy |
| go2_navigation | Required for autonomy |
| go2_perception | Optional |
| go2_bringup | Launch orchestration |

### Base Station

| Package | Priority |
|---------|----------|
| go2_interfaces | **Required** (message types) |
| go2_description | Required (viz) |
| go2_teleop | Required |
| go2_basestation | Launch orchestration |

---

## 9. Safety Classification

| Package | Safety Level | Failure Mode |
|---------|-------------|--------------|
| go2_bridge | **Critical** | Loss = no robot communication |
| go2_motion | **Critical** | Loss = no motion control |
| go2_arbitration | **Critical** | Loss = uncontrolled command sources |
| go2_state | Important | Loss = no fault detection (robot still operates) |
| go2_teleop | Important | Loss = no operator control |
| go2_lidar | Operational | Loss = no SLAM/nav |
| go2_camera | Operational | Loss = no perception |
| go2_localization | Operational | Loss = no autonomous navigation |
| go2_navigation | Operational | Loss = no path planning |
| go2_perception | Optional | Loss = no object detection |
| go2_description | Non-critical | Loss = no visualization |
| go2_bringup | Non-critical | Launch convenience |
| go2_basestation | Non-critical | Monitoring convenience |
| go2_sim | Non-critical | Development convenience |

---

## 10. Implementation Phases

### Phase 1: Foundation (get data flowing)

| Package | Deliverable |
|---------|------------|
| Workspace restructure | Move vendor packages, clean build |
| `go2_interfaces` | Core message/service/action definitions |
| `go2_bridge` | Odom, IMU, joint states, TF, sport_api_call service |
| `go2_description` | Basic URDF, RSP launch |
| `go2_bringup` | `bridge.launch.py` |

**Validation:** `ros2 topic echo /go2/odom`, TF visible in RViz,
robot model displayed with live joint positions.

### Phase 2: Control (move the robot)

| Package | Deliverable |
|---------|------------|
| `go2_motion` | cmd_vel → Move, posture services, gait switching |
| `go2_arbitration` | Teleop mux, watchdog, e-stop |
| `go2_teleop` | Joystick/keyboard → cmd_vel |

**Validation:** drive the robot from base station joystick through
the full arbitration pipeline.

### Phase 3: Sensors (perceive the world)

| Package | Deliverable |
|---------|------------|
| `go2_lidar` | PointCloud filtering, 2D scan extraction |
| `go2_camera` | Front camera calibration, ZED launch integration |
| `go2_bridge` additions | H264 front camera decode, LiDAR pass-through |
| `go2_state` | Health monitoring, fault detection |

**Validation:** LiDAR scan visible in RViz, front camera image displayed,
battery/health data on diagnostics topic.

### Phase 4: Autonomy (navigate autonomously)

| Package | Deliverable |
|---------|------------|
| `go2_localization` | SLAM integration (RTAB-Map or LIO-SAM) |
| `go2_navigation` | Nav2 configuration and launch |
| `go2_basestation` | Full base station stack |

**Validation:** autonomous waypoint following in a mapped environment.

### Phase 5: Polish (production readiness)

| Package | Deliverable |
|---------|------------|
| `go2_perception` | Object detection pipeline |
| `go2_sim` | Simulation environment |
| Rosbag recording | Integrated into basestation launch |

---

## 11. Corrections to Stated Assumptions

### "Motion Services V2.0"

There is no formally versioned "V2.0" in the Unitree SDK2 documentation.
The Sport API accessible via `/api/sport/request` with the `SportClient`
pattern is the current and only high-level motion interface. The API IDs
(1001-2058+) represent individual motion commands. This architecture treats
the Sport API as the canonical motion interface without referencing version
numbers.

### Motion Switcher

The DDS topic `/api/motion_switcher/request` does exist on the robot, but
per your constraint this architecture does not use it. On firmware ~v1.1.11
with secondary development enabled, the sport mode service is available
directly. If future firmware requires explicit service activation, the
`go2_state` node can handle this via the `RobotStateClient.ServiceSwitch()`
pattern (calling `/api/robot_state/request` with api_id 1001).

### Front camera bandwidth

The built-in front camera at 720p/30fps via DDS (`frontvideostream`) over
Ethernet is viable. The data is H264-encoded, so bandwidth is modest (~2-4
Mbps). However, H264 decoding on the Orin should use NVDEC hardware
acceleration rather than CPU decode to avoid consuming CPU budget.

### LiDAR topic type

The `utlidar/cloud` topic publishes `sensor_msgs/PointCloud2` directly
(standard ROS 2 type), not a Unitree-specific type. The bridge can pass
this through with only a frame_id adjustment.

---

## 12. Missing Subsystems (not in original request)

| Subsystem | Purpose | Recommendation |
|-----------|---------|---------------|
| **Time synchronization** | Robot MCU clock vs Orin clock drift | Use `sportmodestate.stamp` to compute offset; apply in go2_bridge |
| **Network health monitor** | Detect Ethernet disconnection | Watchdog in go2_bridge; publish connection status on `/go2/connection` |
| **Parameter management** | Per-deployment configuration | Use ROS 2 parameter files in go2_bringup/config/; consider `rqt_reconfigure` |
| **Map management** | Save/load maps for localization | Service in go2_localization or standalone map_server |
| **Rosbag automation** | Structured recording for research | Launch file in go2_basestation with topic filters |
| **LED control** | Visual status indication on robot | Expose `LowCmd.led` field via go2_bridge service (low priority) |
| **Audio** | Speaker/microphone access | WebRTC-only; not architectured here. Add if needed. |
| **Obstacle avoidance (built-in)** | Firmware obstacle avoidance toggle | Exposed as capability in go2_motion (API 2058 `SwitchAvoidMode`) |
| **Wireless controller deadman switch** | Safety interlock for wireless control | Implement in go2_teleop: require button hold for velocity |
