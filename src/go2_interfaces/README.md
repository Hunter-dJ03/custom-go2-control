# go2_interfaces

Custom ROS 2 message, service, and action definitions for the Go2 system.

## Layer

Cross-cutting — used by all packages in the workspace.

## Purpose

Single source of truth for all custom interface definitions. No nodes, no
logic. Every package that needs Go2-specific types depends on this package
instead of depending on vendor types directly.

## Messages

| Message | Fields | Purpose |
|---------|--------|---------|
| `MotionMode` | uint8 constants (IDLE, STANDING, WALKING, SITTING, EXECUTING_ACTION, DAMP, FAULT) | Enum-like motion mode |
| `MotionState` | mode, gait_type, body_height, foot_raise_height, progress, error_code, capabilities[] | Normalized motion state |
| `ArbitrationStatus` | active_source, priority, safety_state, last_cmd_stamp | Arbitration state |
| `RobotHealth` | battery_soc, battery_voltage, battery_current, motor_temps[12], imu_temp, fault_codes[], uptime | Aggregated health |
| `FootForce` | raw[4], estimated[4] | Foot contact forces |

## Services

| Service | Request | Response | Purpose |
|---------|---------|----------|---------|
| `SetGait` | uint8 gait_type | bool success, string message | Switch locomotion gait |
| `SetPosture` | uint8 posture | bool success, string message | Transition posture |
| `SetBodyHeight` | float32 height | bool success | Adjust body height |
| `SetFootRaiseHeight` | float32 height | bool success | Adjust step height |
| `GetCapabilities` | — | string[] available, string[] unavailable | Query motion capabilities |
| `SetArbitrationSource` | uint8 source | bool success | Switch command source |
| `EStop` | bool engage | bool success | Emergency stop |
| `SportApiCall` | int32 api_id, string parameter_json | bool success, string response_json | Generic sport API pass-through |

## Actions

| Action | Goal | Feedback | Result | Purpose |
|--------|------|----------|--------|---------|
| `ExecuteMotion` | uint8 motion_id, string params_json | float32 progress, string state | bool success, string message | Timed motion execution |

## Dependencies

- `std_msgs`
- `geometry_msgs`
- `builtin_interfaces`
