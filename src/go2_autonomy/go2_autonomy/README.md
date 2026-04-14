# go2_autonomy

Custom SLAM, localization, mapping, and navigation for the Unitree Go2.

**SCAFFOLD — implementation deferred.**

## Planned Components

- SLAM node
- Localization
- Map server
- Nav2 integration
- Costmap management

## Topics

### Publishes
- `/go2/cmd_vel/nav` (`geometry_msgs/Twist`) — navigation velocity commands
- `/go2/map` (`nav_msgs/OccupancyGrid`) — occupancy grid map
- `/go2/pose` (`geometry_msgs/PoseStamped`) — estimated robot pose

## Architecture

All control outputs go through `go2_control` (the control arbiter), not directly to the robot.
