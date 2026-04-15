# custom-go2-control

Custom high-level ROS 2 controller for the Unitree Go2 Pro, running on an
onboard NVIDIA Jetson AGX Orin with ROS 2 Humble.

## Overview

This workspace provides a systems-engineered ROS 2 architecture for
autonomous operation of the Unitree Go2 quadruped. The robot's internal
MCU handles low-level motor control and locomotion. This system handles
everything above that: sensor processing, motion command abstraction,
command arbitration, autonomous navigation, and operator tooling.

Communication with the robot uses CycloneDDS over Ethernet — the same
DDS middleware that ROS 2 uses natively. No SDK wrapper or WebRTC bridge
is required.

## System Architecture

```
Base Station (laptop)                  AGX Orin (onboard)                   Go2 MCU (internal)
┌─────────────────┐    WiFi    ┌──────────────────────────┐   Ethernet   ┌──────────────┐
│ go2_teleop      │◄──────────►│ go2_bridge               │◄────────────►│ Firmware     │
│ go2_basestation │            │ go2_control/             │  CycloneDDS  │ Sport API    │
│ go2_rviz        │            │   go2_motion             │              │ IMU, motors  │
│                 │            │   go2_arbitration         │              │ LiDAR, camera│
│                 │            │   go2_onboard_control     │              └──────────────┘
│                 │            │ go2_hardware/             │
│                 │            │   go2_sensors, lidar, cam │
│                 │            │ go2_autonomy/             │
│                 │            │   slam, localisation,     │
│                 │            │   navigation, perception  │
│                 │            │ go2_state                 │
│                 │            │ go2_description            │
└─────────────────┘            └──────────────────────────┘
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for the full design document including
data flow diagrams, topic/service/action tables, safety classification,
and implementation phasing.

## Workspace Structure

```
src/
├── vendor/                        ← third-party (symlinked, not committed)
│   ├── unitree_go                 → Unitree Go2 message definitions
│   └── unitree_api                → Unitree API message definitions
│
├── go2_interfaces/                ← custom msgs, srvs, actions
├── go2_bridge/                    ← Unitree DDS ↔ standard ROS 2 types
│
├── go2_control/                   ← movement & command group
│   ├── go2_motion/                ← motion abstraction + state machine
│   ├── go2_arbitration/           ← command mux + safety gating
│   ├── go2_teleop/                ← base station joystick/keyboard
│   └── go2_onboard_control/       ← onboard wireless controller + scripts
│
├── go2_hardware/                  ← sensor & hardware group
│   ├── go2_sensors/               ← sensor config + params
│   ├── go2_lidar/                 ← LiDAR processing
│   └── go2_camera/                ← front camera + ZED integration
│
├── go2_autonomy/                  ← autonomy group
│   ├── go2_autonomy/              ← high-level mission logic
│   ├── go2_slam/                  ← map building
│   ├── go2_localisation/          ← localisation in known maps
│   ├── go2_navigation/            ← Nav2 + path planning
│   └── go2_perception/            ← object detection
│
├── go2_state/                     ← health monitoring + diagnostics
├── go2_description/               ← URDF/Xacro robot model
├── go2_rviz/                      ← RViz + Foxglove configs
├── go2_bringup/                   ← onboard launch orchestration
├── go2_basestation/               ← base station launch + viz
└── go2_sim/                       ← simulation (deferred)
```

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `ros-humble-rmw-cyclonedds-cpp`
- `ros-humble-rosidl-generator-dds-idl`

### Unitree ROS 2 Messages

This workspace requires the official Unitree ROS 2 message packages.
Clone and build them first:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2.git ~/unitree_ros2
cd ~/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build
```

Then create symlinks in this workspace:

```bash
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/vendor/unitree_go
ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api src/vendor/unitree_api
```

If the symlinks are missing, `colcon build` will fail with instructions.

## Build

```bash
source ~/go2_ws/setup_env.bash
colcon build
```

## Usage

```bash
# Source the environment
source ~/go2_ws/setup_env.bash

# Verify robot connectivity (should list DDS topics)
ros2 topic list

# Phase 1: robot model + DDS bridge (typical bringup)
ros2 launch go2_bringup phase1.launch.py

# Or run pieces separately:
# ros2 launch go2_description description.launch.py
# ros2 launch go2_bringup bridge.launch.py

# Launch the full onboard stack (placeholder)
ros2 launch go2_bringup robot.launch.py

# Launch the base station (on laptop)
ros2 launch go2_bringup base_station.launch.py
```

## Robot Connection

The Go2 connects to the AGX Orin over Ethernet on the `192.168.123.x`
subnet. CycloneDDS is configured to use the `enp2s0` interface. The
base station connects to the Orin over WiFi.

| Interface | Network | Purpose |
|-----------|---------|---------|
| `enp2s0` (Orin) | 192.168.123.x | Robot ↔ Orin (DDS) |
| WiFi (Orin) | LAN | Orin ↔ Base station (ROS 2) |

## License

Apache-2.0
