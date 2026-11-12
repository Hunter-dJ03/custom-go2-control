#!/usr/bin/env bash
# Build Point-LIO vendor packages inside custom-go2-control.
#
# Usage:
#   ./scripts/build_point_lio_vendor.sh           # build point_lio + unitree_lidar_ros2
#   ./scripts/build_point_lio_vendor.sh --clone-only
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENDOR_WS="$WS_ROOT/src/vendor/ros2_ws/point_lio_ws"
VENDOR_SRC="$VENDOR_WS/src"

clone_only=false
for arg in "$@"; do
  case "$arg" in
    --clone-only) clone_only=true ;;
    *) echo "Unknown argument: $arg" >&2; exit 1 ;;
  esac
done

mkdir -p "$VENDOR_SRC"
cd "$VENDOR_SRC"

if [ ! -d point_lio_ros2/.git ]; then
  git clone https://github.com/dfloreaa/point_lio_ros2.git
fi
if [ ! -d unilidar_sdk2/.git ]; then
  git clone https://github.com/unitreerobotics/unilidar_sdk2.git
fi

if $clone_only; then
  echo "Vendor sources ready under $VENDOR_SRC"
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
set -u

POINT_LIO_WS="$WS_ROOT/src/vendor/ros2_ws/point_lio_ws"
mkdir -p "$POINT_LIO_WS"
cd "$POINT_LIO_WS"
colcon build --symlink-install \
  --packages-select point_lio unitree_lidar_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Build complete. Source the vendor overlay (setup_env.bash does this automatically):"
echo "  source $POINT_LIO_WS/install/setup.bash"
