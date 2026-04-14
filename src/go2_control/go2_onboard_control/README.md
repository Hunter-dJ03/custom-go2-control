# go2_onboard_control

Onboard input handling and direct robot interaction from the Orin.

## Layer

2 — Robot Control

## Purpose

Handles control inputs that originate from the onboard computer itself,
as opposed to `go2_teleop` which handles external operator inputs. This
includes:

- Wireless controller input processing (Unitree remote via `/go2/joy`)
- Button mapping for onboard-triggered actions
- Scripted movement sequences and behaviours
- Programmatic velocity commands from onboard applications

Publishes to `/go2/script/cmd_vel` which flows through `go2_arbitration`
before reaching the robot.

## Nodes

### go2_onboard_control_node

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/joy` | sensor_msgs/Joy | Unitree wireless controller |
| `/go2/motion/state` | go2_interfaces/MotionState | Current robot state |

**Publications:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/script/cmd_vel` | geometry_msgs/Twist | Scripted/onboard velocity commands |

**Service calls:**

| Service | Purpose |
|---------|---------|
| `/go2/motion/set_posture` | Button-mapped posture changes |
| `/go2/motion/execute` | Button-mapped special motions |

## Difference from go2_teleop

| | go2_onboard_control | go2_teleop |
|---|---|---|
| **Runs on** | Onboard (Orin) | Base station (laptop) |
| **Input** | Wireless controller, programmatic | External joystick, keyboard |
| **Publishes** | `/go2/script/cmd_vel` | `/go2/teleop/cmd_vel` |
| **Use case** | Field operation, scripted behaviours | Development, manual override |

## Dependencies

- `go2_interfaces`
- `sensor_msgs`, `geometry_msgs`
- `rclcpp`
