#!/usr/bin/env bash
#
# run_solar_hunter.sh — run the C++ solar-hunter pipeline.
#
# This is the C++ equivalent of solar-hunter/main.py: pick a named aim
# target, compute the reflector pan/tilt angles, optionally run a live
# camera focus-centering loop, and publish the resulting joint angles
# (radians) on /arm_joint_commands (patching only the pan/tilt entries of
# the arm's current joint vector).
#
# Usage:
#   ./run_solar_hunter.sh                              # target=meteors, camera=5
#   ./run_solar_hunter.sh --target locusts --camera 0  # different target/camera
#   ./run_solar_hunter.sh --target meteors --no-camera # aim-only, no camera loop
#   ./run_solar_hunter.sh --build                      # build first if not built
#
# Defaults mirror solar-hunter/main.py:
#   target=meteors   camera_index=4   publish_rate=10 Hz   debug windows on
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_NAME="solar_hunter_cpp"
NODE_BIN="$HERE/install/$PKG_NAME/lib/$PKG_NAME/${PKG_NAME}_node"

# ----- defaults (match solar-hunter/main.py) -----
TARGET="meteors"
CAMERA_INDEX=4
PUBLISH_RATE=10.0
SHOW_DEBUG=true
NO_CAMERA=false
DO_BUILD=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)      TARGET="$2"; shift 2 ;;
    --camera)      CAMERA_INDEX="$2"; shift 2 ;;
    --publish-rate) PUBLISH_RATE="$2"; shift 2 ;;
    --no-camera)   NO_CAMERA=true; shift ;;
    --no-debug)    SHOW_DEBUG=false; shift ;;
    --build)       DO_BUILD=true; shift ;;
    -h|--help)
      echo "Usage: $0 [--target NAME] [--camera IDX] [--publish-rate HZ]"
      echo "       [--no-camera] [--no-debug] [--build]"
      echo ""
      echo "Targets: locusts, meteors, space_debris, moon_south_darkside, solar_sail"
      echo "Note: on this machine camera index 4 (/dev/video4, 'Integrated Camera')"
      echo "      is the working RGB camera; index 5 exists but OpenCV cannot capture it."
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

# ----- source ROS 2 -----
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

# ----- build if requested or binary missing -----
if [[ $DO_BUILD == true || ! -x "$NODE_BIN" ]]; then
  if [[ ! -x "$NODE_BIN" ]]; then
    echo "Binary not found at $NODE_BIN — building..."
  fi
  if [[ -x "$HERE/build_standalone.sh" ]]; then
    "$HERE/build_standalone.sh"
  else
    echo "ERROR: build_standalone.sh not found; cannot build." >&2
    exit 1
  fi
fi

set +u
# shellcheck disable=SC1091
source "$HERE/install/setup.bash"
set -u

# If no camera, force index -1 (aim-only mode).
if [[ $NO_CAMERA == true ]]; then
  CAMERA_INDEX=-1
fi

echo "=== solar_hunter_cpp ==="
echo "  target:       $TARGET"
echo "  camera:       $CAMERA_INDEX  (-1 = aim-only, no camera loop)"
echo "  publish_rate: $PUBLISH_RATE Hz"
echo "  debug:        $SHOW_DEBUG"
echo ""

exec ros2 run "$PKG_NAME" "${PKG_NAME}_node" \
  --ros-args \
  -p target:="$TARGET" \
  -p camera_index:="$CAMERA_INDEX" \
  -p publish_rate_hz:="$PUBLISH_RATE" \
  -p show_debug_windows:="$SHOW_DEBUG"
