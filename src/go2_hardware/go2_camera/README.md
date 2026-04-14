# go2_camera

Camera data management and integration.

## Layer

4 — Sensors

## Purpose

Manages camera calibration and integration for:
- Built-in front camera (H264 decoded by go2_bridge, calibration published here)
- ZED camera (launched via zed_ros2_wrapper, configured and namespaced here)

## Nodes

### go2_front_camera_info_node

Publishes CameraInfo for the front camera to pair with the decoded image
from go2_bridge.

| Topic | Type |
|-------|------|
| `/go2/front_camera/camera_info` | sensor_msgs/CameraInfo |

### ZED integration

No custom node. Launch files configure and start `zed_ros2_wrapper` with
appropriate remappings into the `/go2/` namespace.

## Files

```
go2_camera/
├── config/
│   ├── front_camera_720.yaml    ← front camera calibration
│   └── zed_params.yaml          ← ZED camera parameters
├── launch/
│   ├── front_camera.launch.py
│   └── zed.launch.py
└── README.md
```

## Dependencies

- `sensor_msgs`
- `camera_info_manager`
- `zed_ros2_wrapper` (optional, for ZED)
