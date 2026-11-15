#!/usr/bin/env bash
#
# build_standalone.sh — build solar_hunter_cpp as a standalone ament_cmake
# package outside of any ROS 2 workspace.
#
# Because the package uses rclcpp / sensor_msgs / OpenCV it still needs a
# ROS 2 installation to compile against, but it no longer needs to live
# inside ros2_ws/src/.
#
# Usage:
#   ./build_standalone.sh               # build + install into ./install/
#   ./build_standalone.sh --clean       # wipe build/ and install/ first
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_NAME="solar_hunter_cpp"

CLEAN=0
if [[ "${1:-}" == "--clean" ]]; then
  CLEAN=1
fi

# Source ROS 2 (try common distro locations).
# ROS setup.bash may reference unbound variables, so relax nounset around it.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for ds in /opt/ros/*/setup.bash; do
    if [[ -f "$ds" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "$ds"
      set -u
      break
    fi
  done
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ERROR: could not source a ROS 2 setup.bash (set ROS_DISTRO or install ROS 2)." >&2
  exit 1
fi
echo "ROS 2 distro: $ROS_DISTRO"

# Colcon expects the package under <ws>/src/<pkg>/. Build an isolated
# colcon workspace under ./build/colcon_ws/ that symlinks this package.
WS="$HERE/build/colcon_ws"
SRC="$WS/src/$PKG_NAME"

if [[ $CLEAN -eq 1 ]]; then
  echo "Cleaning build/ and install/ ..."
  rm -rf "$HERE/build" "$HERE/install" "$HERE/log"
fi

mkdir -p "$WS/src"
if [[ ! -e "$SRC" ]]; then
  ln -s "$HERE" "$SRC"
fi

echo "Building $PKG_NAME in isolated colcon workspace ..."
set +u
# shellcheck disable=SC1091
source /opt/ros/"$ROS_DISTRO"/setup.bash 2>/dev/null || true
set -u
colcon build --packages-select "$PKG_NAME" --base-paths "$WS" \
  --install-base "$HERE/install" --build-base "$HERE/build" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Build complete. To use:"
echo "  source $HERE/install/setup.bash"
echo "  ros2 run $PKG_NAME ${PKG_NAME}_node --help"
