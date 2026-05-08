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

SolidWorks-derived link tree (`base_link`, `odom`, `map`, leg links,
`imu_link`, cameras per upstream URDF).

## Dependencies

`robot_state_publisher`, `joint_state_publisher`, `urdf`
