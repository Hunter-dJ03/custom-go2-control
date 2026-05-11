#!/bin/bash
# Go2 Robot Environment Setup
# Source this before ros2 launch (and before colcon build — packages need
# unitree_go / unitree_api on CMAKE_PREFIX_PATH):
#   source ~/unitree_ws/custom-go2-control/setup_env.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
GO2_ETH="${GO2_ETH:-eno1}"

# --- ROS2 base ---
source /opt/ros/humble/setup.bash

# --- Unitree ROS2 messages (unitree_go, unitree_api); lives under ~/unitree_ws ---
if [ -f "$UNITREE_WS/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$UNITREE_WS/unitree_ros2/cyclonedds_ws/install/setup.bash"
elif [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

# --- This workspace overlay ---
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# --- CycloneDDS configuration ---
# Dual-interface config: WiFi (wlP1p1s0) primary for base station SEDP/data,
# eno1 secondary for Go2 robot multicast. Both interfaces have multicast=true
# with MulticastRecvNetworkInterfaceAddresses=all.
# See ~/.config/go2/README.md for full documentation.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/.config/go2/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file://${HOME}/.config/go2/cyclonedds.xml"
else
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"${GO2_ETH}\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>"
fi

# --- Multicast route for DDS discovery on robot Ethernet ---
if ! ip route show | grep -q "224.0.0.0/4 dev ${GO2_ETH}"; then
    sudo ip route add multicast 224.0.0.0/4 dev "${GO2_ETH}" 2>/dev/null && \
        echo "  Multicast route added for ${GO2_ETH}" || \
        echo "  WARNING: Failed to add multicast route (try sudo or set GO2_ETH to your robot NIC)"
fi

# --- Python: optional venv site-packages + block ~/.local (NumPy mismatch) ---
for _VENV_SP in "$HOME/go2_ros2_sdk_vendor/.venv/lib/python3.10/site-packages"; do
    if [ -d "$_VENV_SP" ]; then
        export PYTHONPATH="$_VENV_SP${PYTHONPATH:+:$PYTHONPATH}"
        break
    fi
done
export PYTHONNOUSERSITE=1

# --- Robot connection defaults ---
export ROBOT_IP="${ROBOT_IP:-192.168.123.161}"

echo "Go2 environment loaded:"
echo "  RMW:            $RMW_IMPLEMENTATION"
echo "  ROBOT_IP:       $ROBOT_IP"
echo "  Workspace:      $SCRIPT_DIR"
