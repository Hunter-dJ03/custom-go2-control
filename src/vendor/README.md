# vendor/

Third-party packages. Do not modify.

## Setup

These packages are **not** committed to the repo. You must create symlinks
to your local `unitree_ros2` installation:

```bash
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/vendor/unitree_go
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api src/vendor/unitree_api
```

For the Go2 meshes and SolidWorks-exported URDF used by `go2_description`, link
the [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) repository
(for example from `unitree_ws`):

```bash
ln -sfn ~/unitree_ws/src/go2_ros2_sdk src/vendor/go2_ros2_sdk
```

If the symlinks are missing, `colcon build` will fail with instructions.

## Packages

| Package | Source | Purpose |
|---------|--------|---------|
| `unitree_go` | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | Unitree Go2 message definitions (24 msg types) |
| `unitree_api` | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | Unitree API message definitions (8 msg types) |
| `go2_ros2_sdk` | [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | Go2 URDF + DAE meshes (consumed at build time by `go2_description`; not built as a workspace package) |

## Policy

- Only `go2_bridge` should depend on `unitree_go` / `unitree_api` as compile-time ROS packages
- `go2_description` reads `go2_ros2_sdk` only at **CMake configure** time (URDF + meshes are installed into `go2_description`); nothing should `find_package` or `exec_depend` on it at runtime
- All other packages use standard ROS 2 types + `go2_interfaces`
- Update by pulling latest from upstream and rebuilding
