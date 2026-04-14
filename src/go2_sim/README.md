# go2_sim

Simulation support for development and testing.

## Layer

6 — Description & Simulation

## Status

**DEFERRED** — implement after core stack is validated (Phase 5).

## Purpose

Simulation environment (Gazebo, Isaac Sim, or MuJoCo) that implements
the same interfaces as the real robot. The key design goal: `go2_motion`
and `go2_arbitration` work identically in sim and real deployment.
Only the bridge layer is swapped.

## Planned approach

- Sim bridge node that mimics `go2_bridge` interface
- Physics sim for Go2 body and leg dynamics
- Simulated LiDAR and camera sensors

## Dependencies (planned)

- `go2_interfaces`, `go2_description`
- Simulation framework package (Gazebo/Isaac)
