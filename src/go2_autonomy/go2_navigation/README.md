# go2_navigation

Path planning and autonomous navigation.

## Layer

5 — Autonomy

## Status

**DEFERRED** — implement after localization is working (Phase 4).

## Purpose

Nav2 integration for autonomous waypoint following and path planning.
Outputs velocity commands through the arbitration layer — never directly
to the robot.

## Publications

| Topic | Type | Purpose |
|-------|------|---------|
| `/go2/nav/cmd_vel` | geometry_msgs/Twist | Planned velocity → arbitration |

## Key design constraint

Navigation output goes to `/go2/nav/cmd_vel`, NOT `/go2/cmd_vel`.
The arbitration node is the only publisher on `/go2/cmd_vel`.

## Dependencies (planned)

- `nav2_bringup`, `nav2_bt_navigator`, etc. (apt packages)
- `go2_interfaces`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
