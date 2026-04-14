# go2_basestation

Base station launch, visualization, and recording.

## Layer

7 — Operator & Tooling

## Purpose

Launch files for the base station computer. Runs on the operator laptop
connected to the onboard Orin over WiFi. No safety-critical functions.

## Structure

```
go2_basestation/
├── launch/
│   ├── basestation.launch.py   ← RViz + teleop + Foxglove
│   ├── record.launch.py        ← rosbag recording with topic filters
│   └── playback.launch.py      ← rosbag playback + visualization
├── config/
│   ├── foxglove_bridge.yaml
│   └── go2.rviz
└── README.md
```

## Runs on

Base station laptop only.

## Dependencies

- `go2_description`, `go2_interfaces`, `go2_teleop`
- `foxglove_bridge`
- `rosbag2_transport`
