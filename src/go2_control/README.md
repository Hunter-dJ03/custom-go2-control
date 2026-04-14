# go2_control

Robot control, command arbitration, and teleoperation packages.

All velocity and motion commands flow through this layer before reaching
the robot. Nothing bypasses the arbitration node.

## Packages

| Package | Purpose |
|---------|---------|
| `go2_motion` | Unified motion abstraction (cmd_vel → Sport API, gait/posture control, capability queries) |
| `go2_arbitration` | Priority-based command mux, safety gating, watchdog, e-stop |
| `go2_teleop` | Joystick and keyboard teleoperation (base station) |
| `go2_onboard_control` | Wireless controller input, scripted behaviours (onboard) |

## Command Flow

```
go2_teleop ──────────► /go2/teleop/cmd_vel ──┐
                                             │
go2_onboard_control ──► /go2/script/cmd_vel ─┤
                                             ├──► go2_arbitration ──► /go2/cmd_vel ──► go2_motion ──► robot
go2_navigation ──────► /go2/nav/cmd_vel ─────┤
                                             │
other scripts ───────► /go2/script/cmd_vel ──┘
```
