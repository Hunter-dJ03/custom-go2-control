# go2_state

Robot state aggregation, health monitoring, and fault detection.

## Layer

3 — Robot State & Health

## Purpose

Separates high-rate control-relevant state (handled by go2_bridge at 50 Hz)
from low-rate diagnostics. Aggregates health data from multiple sources and
publishes unified health/diagnostic summaries. Monitors for fault conditions
and can trigger safety responses via arbitration.

## Nodes

### go2_state_node

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/battery` | sensor_msgs/BatteryState | Battery monitoring |
| `/go2/joint_states` | sensor_msgs/JointState | Motor temperature |
| `/go2/motion/state` | go2_interfaces/MotionState | Error code tracking |
| `/go2/odom` | nav_msgs/Odometry | Liveness check |

**Publications:**

| Topic | Type | Rate |
|-------|------|------|
| `/go2/health` | go2_interfaces/RobotHealth | 1 Hz |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 1 Hz |

## Fault Thresholds

| Condition | Action |
|-----------|--------|
| Battery SOC < 15% | Warning on /diagnostics |
| Battery SOC < 5% | Trigger sit-down via e-stop |
| Motor temp > 70°C | Warning |
| Motor temp > 85°C | Trigger damp |
| No odom for > 2s | Connection loss alarm |
| error_code != 0 | Parse and publish fault |

## Safety

Important — loss means no fault detection, but robot still operates.

## Dependencies

- `go2_interfaces`
- `sensor_msgs`, `nav_msgs`, `diagnostic_msgs`
- `rclcpp`
