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
│   ├── robot.launch.py         ← full onboard stack
│   ├── bridge.launch.py        ← bridge + state only (testing)
│   ├── control.launch.py       ← motion + arbitration
│   ├── sensors.launch.py       ← LiDAR + cameras
│   └── navigation.launch.py   ← SLAM + Nav2
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
# Source environment
source ~/unitree_ws/setup_env.bash

# Full stack
ros2 launch go2_bringup robot.launch.py

# Bridge only (testing)
ros2 launch go2_bringup bridge.launch.py
```

## Dependencies (exec_depend)

All `go2_*` packages.
