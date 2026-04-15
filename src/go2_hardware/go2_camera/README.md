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
the robot's Ethernet interface. This node uses OpenCV's GStreamer backend to
receive, decode, and convert frames to BGR, then publishes them as standard
ROS 2 image messages.

Reference: https://support.unitree.com/home/en/developer/Multimedia_Services

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
