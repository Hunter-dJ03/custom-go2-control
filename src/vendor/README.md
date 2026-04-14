# vendor/

Third-party packages. Do not modify.

## Setup

These packages are **not** committed to the repo. You must create symlinks
to your local `unitree_ros2` installation:

```bash
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/vendor/unitree_go
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api src/vendor/unitree_api
```

If the symlinks are missing, `colcon build` will fail with instructions.

## Packages

| Package | Source | Purpose |
|---------|--------|---------|
| `unitree_go` | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | Unitree Go2 message definitions (24 msg types) |
| `unitree_api` | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | Unitree API message definitions (8 msg types) |

## Policy

- Only `go2_bridge` should depend on these packages
- All other packages use standard ROS 2 types + `go2_interfaces`
- Update by pulling latest from upstream and rebuilding
