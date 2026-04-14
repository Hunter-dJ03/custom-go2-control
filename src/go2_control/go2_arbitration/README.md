# go2_arbitration

Priority-based command multiplexer and safety gatekeeper.

## Layer

2 — Robot Control Abstraction

## Purpose

**The single point of authority for all robot commands.** All command sources
(teleop, autonomy, scripted behaviors) must pass through arbitration before
reaching the motion layer. Implements:

- Priority-based command muxing
- Watchdog timeout (zero velocity on command loss)
- Emergency stop (immediate Damp)
- Fault response (reject commands during fault, attempt recovery)

Modeled after the `joint_desired_control_mux` pattern from the rover
architecture, extended with priority levels and safety monitoring.

## Nodes

### go2_arbitration_node

**Subscriptions (command inputs):**

| Topic | Type | Source | Priority |
|-------|------|--------|----------|
| `/go2/teleop/cmd_vel` | geometry_msgs/Twist | go2_teleop | 2 (medium) |
| `/go2/nav/cmd_vel` | geometry_msgs/Twist | go2_navigation | 1 (low) |
| `/go2/script/cmd_vel` | geometry_msgs/Twist | scripted behaviors | 1 (low) |

**Subscriptions (state):**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/motion/state` | go2_interfaces/MotionState | Fault detection |

**Publications:**

| Topic | Type | Rate |
|-------|------|------|
| `/go2/cmd_vel` | geometry_msgs/Twist | 10 Hz |
| `/go2/arbitration/status` | go2_interfaces/ArbitrationStatus | 5 Hz |

**Services:**

| Service | Type | Purpose |
|---------|------|---------|
| `/go2/arbitration/set_source` | SetArbitrationSource | Switch active source |
| `/go2/arbitration/estop` | EStop | Emergency stop |

## Safety Rules

1. E-Stop (priority 255): always wins, sends Damp immediately
2. Teleop (priority 2): operator override, preempts autonomy
3. Autonomy/Script (priority 1): normal operation
4. Watchdog: zero velocity after 500ms timeout, StopMove after 1500ms
5. Fault: reject all commands, trigger RecoveryStand once

## Safety

**CRITICAL** — the single gatekeeper for all robot commands.

## Dependencies

- `go2_interfaces`
- `geometry_msgs`
- `rclcpp`
