# go2_bringup

Onboard launch orchestration. No nodes — launch files and config only.

## Layer

7 — Operator & Tooling

## Purpose

Following the kanga_bringup pattern: thin orchestration layer that
composes launches from other packages. Contains parameter files for
deployment configuration.

## Structure

```
go2_bringup/
├── launch/
│   ├── robot.launch.py         ← full onboard stack (placeholder)
│   ├── phase1.launch.py        ← description + bridge + Foxglove (optional RViz)
│   ├── foxglove_bridge.launch.py ← Foxglove WebSocket bridge only
│   ├── bridge.launch.py        ← DDS bridge only
│   ├── base_station.launch.py
│   ├── sensors.launch.py
│   └── teleop.launch.py
├── config/
│   ├── bridge_params.yaml
│   ├── motion_params.yaml
│   ├── arbitration_params.yaml
│   ├── nav2_params.yaml
│   └── cyclonedds.xml
└── README.md
```

## Usage

```bash
# Source environment (Humble + unitree_ros2 + this workspace)
source ~/go2_ws/setup_env.bash

# Phase 1: go2_description (RSP) + go2_bridge + foxglove_bridge (Foxglove Studio on ws://127.0.0.1:8765)
ros2 launch go2_bringup phase1.launch.py

# RViz instead of Foxglove, or bridge only:
# ros2 launch go2_bringup phase1.launch.py foxglove:=false rviz:=true
# ros2 launch go2_bringup foxglove_bridge.launch.py

# DDS bridge only (e.g. with description launched elsewhere)
ros2 launch go2_bringup bridge.launch.py

# Full stack (placeholder)
ros2 launch go2_bringup robot.launch.py
```

## Dependencies (exec_depend)

`go2_bridge`, `go2_description`, `robot_state_publisher`, `launch_ros`, `foxglove_bridge` (apt: `ros-humble-foxglove-bridge`).
