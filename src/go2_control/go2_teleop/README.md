# go2_teleop

Teleoperation interfaces for manual robot control.

## Layer

7 — Operator & Tooling

## Purpose

Maps joystick/keyboard input to velocity commands and motion service calls.
Supports two input paths:

1. **Unitree wireless controller** (onboard): `/go2/joy` from go2_bridge
2. **External joystick/keyboard** (base station): standard `/joy` topic

## Nodes

### go2_teleop_node

**Subscriptions:**

| Topic | Type | Source |
|-------|------|--------|
| `/go2/joy` | sensor_msgs/Joy | Unitree wireless controller |
| `/joy` | sensor_msgs/Joy | External joystick |

**Publications:**

| Topic | Type |
|-------|------|
| `/go2/teleop/cmd_vel` | geometry_msgs/Twist |

**Service calls:**

| Service | Purpose |
|---------|---------|
| `/go2/motion/set_posture` | Button-mapped posture changes |
| `/go2/motion/execute` | Button-mapped special motions |
| `/go2/arbitration/estop` | Emergency stop button |

## Safety

Important — operator control path. Should implement deadman switch
(require button hold for velocity output).

## Dependencies

- `go2_interfaces`
- `sensor_msgs`, `geometry_msgs`
- `rclcpp`
