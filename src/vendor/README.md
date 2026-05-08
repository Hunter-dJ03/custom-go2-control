# vendor/

Optional **reference** copies of Unitree message sources. This directory has
**`COLCON_IGNORE`** so `colcon` never descends here (avoids rebuilding
`unitree_*` in this overlay).

Symlinks are optional and **not** inputs to `find_package` — compile-time
`unitree_go` / `unitree_api` come only from sourcing
`unitree_ros2/cyclonedds_ws/install/setup.bash`.

```bash
ln -sf ~/unitree_ws/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/vendor/unitree_go
ln -sf ~/unitree_ws/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api src/vendor/unitree_api
```

## Longer-term

- **`unitree_*`**: Dropping them means maintaining your own `.msg`/`idl` codegen
  or isolating all includes to a single bridge package.

Robot URDF/meshes live in-tree under **`go2_description/{urdf,meshes}`**
(vendored files, no `go2_ros2_sdk` repo required).
