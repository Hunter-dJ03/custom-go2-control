# go2_description

Robot model (URDF), meshes, and kinematic configuration for the Unitree Go2.

## Source of the model

URDF and Collada meshes are **stored in-tree** under `urdf/` and `meshes/`
(origin: community `go2_ros2_sdk`, BSD license). Mesh URLs already point at
`package://go2_description/meshes/`, so CMake just installs the files as-is to
`share/go2_description/{urdf,meshes}/`.

No external `go2_ros2_sdk` checkout or `GO2_ROS2_SDK_PATH` is required.

## Structure

```
go2_description/
├── urdf/
│   └── go2.urdf
├── meshes/
│   └── *.dae
├── launch/
│   └── description.launch.py          ← robot_state_publisher + URDF
├── config/
│   └── joint_names.yaml
└── CMakeLists.txt
```

Launch the model alone:

```bash
ros2 launch go2_description description.launch.py
```

## Key frames

SolidWorks-derived link tree (`base_footprint` → `base_link`, legs, bodies,
cameras — see `go2.urdf`).

**Important:** **`map`** and **`odom`** were **removed** from this URDF (they appeared as fixed
SolidWorks-export joints only). **`odom` → `base_link`** is published by **`go2_bridge`**.
**`map` → `odom`** comes from **`map_odom_tf_node`** (identity placeholder via
`go2_localisation`'s LaunchArg `map_odom_identity_tf:=true`) **or from SLAM** (dynamic).
Do not put those world-frame links back into **`robot_description`**, or you will duplicate
static TF versus the localization stack.

## Dependencies

`robot_state_publisher`, `joint_state_publisher`, `urdf`
