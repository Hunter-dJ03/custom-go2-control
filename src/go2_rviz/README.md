# go2_rviz

Visualization configuration files for RViz2 and Foxglove.

## Available Configs

| Config | Fixed Frame | Displays |
|--------|-------------|----------|
| `go2_default.rviz` | `map` | RobotModel, TF, PointCloud2 (LiDAR), Image (front camera), Odometry |
| `go2_nav.rviz` | `map` | RobotModel, TF, Map, Path, LaserScan, Odometry |

## Usage

```bash
rviz2 -d $(ros2 pkg prefix go2_rviz)/share/go2_rviz/rviz/go2_default.rviz
```

```bash
rviz2 -d $(ros2 pkg prefix go2_rviz)/share/go2_rviz/rviz/go2_nav.rviz
```

## Foxglove

The `foxglove/` directory is reserved for Foxglove Studio layout files.
