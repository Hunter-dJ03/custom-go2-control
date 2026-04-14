# go2_perception

Perception pipeline for object detection and scene understanding.

## Layer

5 — Autonomy

## Status

**DEFERRED** — implement after sensor pipeline is validated (Phase 5).

## Purpose

Consumes camera and LiDAR data, produces detections and classifications
for use by the navigation and autonomy layers.

## Planned capabilities

- Object detection (YOLO or similar on Orin GPU)
- Terrain classification
- Obstacle segmentation from point clouds

## Dependencies (planned)

- `sensor_msgs`
- `vision_msgs`
- `go2_interfaces`
