#!/bin/bash
# Go2 Robot Environment Setup
# Source this before running ros2 launch:
#   source ~/go2_ws/setup_env.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- ROS2 base ---
source /opt/ros/humble/setup.bash

# --- Unitree ROS2 messages (unitree_go, unitree_api) ---
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

# --- This workspace overlay ---
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# --- CycloneDDS configuration ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp2s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

# --- Python: use vendor venv, block ~/.local (NumPy 2.x) ---
_VENV_SP="$HOME/go2_ros2_sdk_vendor/.venv/lib/python3.10/site-packages"
if [ -d "$_VENV_SP" ]; then
    export PYTHONPATH="$_VENV_SP${PYTHONPATH:+:$PYTHONPATH}"
fi
export PYTHONNOUSERSITE=1

# --- Robot connection defaults ---
export ROBOT_IP="${ROBOT_IP:-192.168.123.161}"

echo "Go2 environment loaded:"
echo "  RMW:            $RMW_IMPLEMENTATION"
echo "  ROBOT_IP:       $ROBOT_IP"
echo "  Workspace:      $SCRIPT_DIR"
