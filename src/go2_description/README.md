# go2_description

Robot model (URDF/Xacro), meshes, and robot_state_publisher launch.

## Layer

6 — Description & Simulation

## Purpose

Defines the physical model of the Go2 for TF tree generation and
visualization. Following the kanga_description two-layer xacro pattern:
macro files define components, assembly files compose them.

## Structure

```
go2_description/
├── urdf/
│   ├── go2_macro.urdf.xacro       ← body + 4 legs (12 joints, visual only)
│   ├── go2.urdf.xacro             ← full assembly with sensors
│   ├── sensors/
│   │   ├── lidar.urdf.xacro       ← Unitree L2 mount
│   │   ├── front_camera.urdf.xacro
│   │   └── zed.urdf.xacro         ← ZED camera mount
│   └── payloads/
│       └── orin_payload.urdf.xacro
├── meshes/                         ← visual meshes (.dae or .stl)
├── config/
│   └── joint_names.yaml
├── launch/
│   └── rsp.launch.py              ← robot_state_publisher
└── rviz/
    └── go2.rviz                   ← default RViz config
```

## Key frames

`base_link`, `imu_link`, `lidar_link`, `front_camera_link`,
`zed_camera_link`, `FL_hip`, `FL_thigh`, `FL_calf`, `FL_foot`, etc.

## Notes

- No transmissions defined (we do not do low-level motor control)
- Joint states come from `/go2/joint_states` published by go2_bridge
- Static sensor transforms defined in URDF

## Dependencies

- `robot_state_publisher`, `xacro`, `joint_state_publisher`
- `urdf`
