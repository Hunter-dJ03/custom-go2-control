# go2_camera

Front camera driver for the Unitree Go2.

## Layer

4 — Sensors

## Purpose

Receives H.264 video from the Go2's built-in front camera via UDP multicast
(GStreamer pipeline) and publishes both raw and JPEG-compressed images to
ROS 2 topics.

## How it works

The Go2 MCU streams H.264 video over UDP multicast at `230.1.1.1:1720` on
the robot's Ethernet interface. The node builds a GStreamer pipeline (decode
to BGR in `appsink`), then publishes `sensor_msgs/Image` via `cv_bridge` and
optional JPEG `CompressedImage` via OpenCV `imencode`.

Reference: https://support.unitree.com/home/en/developer/Multimedia_Services

## TODO — when deploying on Orin (onboard autonomy priority)

Base station is for monitoring only; optimize the NVIDIA side for perception
and control headroom.

- [ ] **Hardware H.264 decode** — Replace `avdec_h264` with a Jetson hardware
  decoder (e.g. `nvv4l2decoder`; exact element depends on JetPack/GStreamer)
  to cut CPU use versus software decode.
- [ ] **Frame pump from GStreamer** — Drive publishing from `appsink` (callback
  or blocking pull) instead of a wall timer + `try_pull_sample(0)` so decoded
  frames are not dropped by timing.
- [ ] **Limit full-rate raw fan-out** — Keep full BGR `image_raw` for onboard
  consumers that need it; use compressed or a capped **preview rate** for RViz /
  Wi‑Fi to the base station.
- [ ] **Isolate load from control** — Consider process/thread priority or cgroup
  CPU so camera/perception spikes do not starve `go2_bridge` or the controller.

## Nodes

### go2_camera_node

| Published Topic | Type | Description |
|-----------------|------|-------------|
| `/go2/front_camera/image_raw` | `sensor_msgs/Image` | Raw BGR8 image at 1280x720 |
| `/go2/front_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG-compressed image |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `multicast_address` | `230.1.1.1` | Multicast group address |
| `multicast_port` | `1720` | UDP port for video stream |
| `network_interface` | `enp2s0` | Network interface connected to Go2 MCU |
| `width` | `1280` | Output image width |
| `height` | `720` | Output image height |
| `target_fps` | `30` | Target frame rate |
| `jpeg_quality` | `80` | JPEG compression quality (0-100) |
| `frame_id` | `front_camera_link` | TF frame ID for image headers |

## Prerequisites

- OpenCV built with GStreamer support
- GStreamer plugins: `gstreamer1.0-plugins-base`, `gstreamer1.0-plugins-good`,
  `gstreamer1.0-plugins-bad`, `gstreamer1.0-libav`
- Network access to Go2 MCU on `192.168.123.x`

## Usage

```bash
# Standalone
ros2 run go2_camera camera_node --ros-args \
  -p network_interface:=enp2s0

# Via launch file
ros2 launch go2_camera camera.launch.py network_interface:=enp2s0

# Via bringup
ros2 launch go2_bringup sensors.launch.py front_camera:=true
```
