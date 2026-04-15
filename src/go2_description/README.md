# go2_description

Robot model (URDF/Xacro), meshes, and kinematic configuration for the Unitree Go2.

## Source of the model

At **build** time, CMake reads
`../vendor/go2_ros2_sdk/go2_robot_sdk/urdf/go2.urdf` and the `dae/` meshes from
the same tree, rewrites mesh URLs to `package://go2_description/meshes/`, and
installs the result as `share/go2_description/urdf/go2.urdf.xacro` plus the
`.dae` files under `share/go2_description/meshes/`. Create the vendor symlink
first (see `src/vendor/README.md`).

## Structure

```
go2_description/
├── launch/
│   └── description.launch.py      ← robot_state_publisher + URDF
├── urdf/
│   └── README.txt                 ← points to CMake-generated install artifact
├── config/
│   └── joint_names.yaml
└── CMakeLists.txt                 ← generates URDF + installs meshes from vendor
```

Launch the model alone:

```bash
ros2 launch go2_description description.launch.py
```

## Key frames

Includes the full SolidWorks-exported tree from go2_ros2_sdk (`base_link`,
`odom`, `map`, leg links, `imu_link`, cameras, etc. as defined upstream).

## Dependencies

`robot_state_publisher`, `xacro`, `joint_state_publisher`, `urdf`
