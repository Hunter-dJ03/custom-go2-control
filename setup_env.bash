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

# --- Point-LIO vendor overlay (built separately; src/vendor/ has COLCON_IGNORE) ---
POINT_LIO_INSTALL="$SCRIPT_DIR/src/vendor/ros2_ws/point_lio_ws/install/setup.bash"
if [ -f "$POINT_LIO_INSTALL" ]; then
    source "$POINT_LIO_INSTALL"
fi

# --- CycloneDDS configuration ---
# Dual-interface in principle: WiFi (wlP1p1s0) primary for base station SEDP/data,
# robot Ethernet (${GO2_ETH}, default eno1) secondary for Go2 robot multicast.
# CycloneDDS 0.10 (Humble) refuses to create a domain if ANY listed <NetworkInterface>
# is missing or has no carrier — so we generate the XML at session time, only
# listing interfaces that are actually UP. The static ~/.config/go2/cyclonedds.xml
# is no longer sourced directly; it's used as a template / reference only.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

GO2_WIFI="${GO2_WIFI:-wlP1p1s0}"
_GO2_DDS_RUNTIME="${XDG_RUNTIME_DIR:-/tmp}/go2_cyclonedds.${USER}.xml"

_iface_up() {
    # An interface is "usable" only if it exists AND has carrier (lower-up).
    ip -o link show "$1" 2>/dev/null | grep -q "state UP"
}

_go2_dds_interfaces=""
_iface_up "$GO2_WIFI" && _go2_dds_interfaces+="        <NetworkInterface name=\"$GO2_WIFI\" priority=\"default\" multicast=\"true\"/>
"
_iface_up "$GO2_ETH"  && _go2_dds_interfaces+="        <NetworkInterface name=\"$GO2_ETH\" priority=\"default\" multicast=\"true\"/>
"

if [ -z "$_go2_dds_interfaces" ]; then
    # Last-resort fallback: let Cyclone pick the best interface itself.
    _go2_dds_interfaces='        <NetworkInterface autodetermine="true" priority="default" multicast="true"/>
'
    echo "  WARNING: neither $GO2_WIFI nor $GO2_ETH is up — using autodetermine"
fi

cat > "$_GO2_DDS_RUNTIME" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
$_go2_dds_interfaces      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MulticastRecvNetworkInterfaceAddresses>all</MulticastRecvNetworkInterfaceAddresses>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.1"/>
        <Peer address="192.168.123.161"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
export CYCLONEDDS_URI="file://$_GO2_DDS_RUNTIME"
unset _go2_dds_interfaces

# --- Multicast route for DDS discovery on robot Ethernet (only if eno1 is up) ---
if _iface_up "$GO2_ETH"; then
    if ! ip route show | grep -q "224.0.0.0/4 dev ${GO2_ETH}"; then
        sudo ip route add multicast 224.0.0.0/4 dev "${GO2_ETH}" 2>/dev/null && \
            echo "  Multicast route added for ${GO2_ETH}" || \
            echo "  WARNING: Failed to add multicast route (try sudo or set GO2_ETH to your robot NIC)"
    fi
fi
unset -f _iface_up

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
