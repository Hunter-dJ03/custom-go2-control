# go2_motion

Unified high-level motion abstraction built on the Unitree Sport API.

## Layer

2 — Robot Control Abstraction

## Purpose

Owns all motion semantics: API ID constants, JSON parameter encoding,
motion state machine, capability abstraction. Translates standard ROS 2
velocity commands and service calls into Unitree Sport API requests via
the `go2_bridge` service interface.

- Velocity control (cmd_vel → Sport API Move)
- Gait switching (trot, run, climb, static, economic)
- Posture transitions (stand, sit, damp, balance, recovery)
- Special motions via action server (hello, stretch, flip, dance)
- Capability queries (firmware-dependent feature availability)

## Nodes

### go2_motion_node

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/cmd_vel` | geometry_msgs/Twist | Velocity commands from arbitration |
| `/go2/motion/raw_state` | go2_interfaces/MotionState | Current robot mode tracking |

**Publications:**

| Topic | Type | Rate |
|-------|------|------|
| `/go2/motion/state` | go2_interfaces/MotionState | 10 Hz |

**Services:**

| Service | Type |
|---------|------|
| `/go2/motion/set_gait` | SetGait |
| `/go2/motion/set_posture` | SetPosture |
| `/go2/motion/set_body_height` | SetBodyHeight |
| `/go2/motion/set_foot_raise_height` | SetFootRaiseHeight |
| `/go2/motion/get_capabilities` | GetCapabilities |

**Actions:**

| Action | Type |
|--------|------|
| `/go2/motion/execute` | ExecuteMotion |

## Internal State Machine

States: IDLE → STANDING → WALKING → SITTING → EXECUTING_ACTION → DAMP → FAULT

Key rules:
- Must be STANDING before WALKING
- Special motions set EXECUTING_ACTION and reject velocity until complete
- FAULT triggers automatic RecoveryStand attempt

## Safety

**Critical** — commands reach the robot through this node.

## Dependencies

- `go2_interfaces`
- `geometry_msgs`
- `rclcpp`, `rclcpp_action`
