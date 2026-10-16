#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/lyrical/setup.bash

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPTH_SCRIPT="$SCRIPT_DIR/pobs_depth_sensor.py"

if [[ ! -f "$DEPTH_SCRIPT" ]]; then
  echo "Missing depth script: $DEPTH_SCRIPT" >&2
  echo "Put pobs_depth_sensor.py beside this script." >&2
  exit 1
fi

BRIDGE_PID=""
DEPTH_PID=""

cleanup() {
  trap - EXIT INT TERM

  for pid in "$BRIDGE_PID" "$DEPTH_PID"; do
    if [[ -n "$pid" ]]; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting POBS sensor and pose bridges..."

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/pobs/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/pobs/sonar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/model/pobs/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' &
BRIDGE_PID=$!

echo "Starting POBS depth and pressure publisher..."

python3 "$DEPTH_SCRIPT" &
DEPTH_PID=$!

echo "POBS topics:"
echo "  /clock"
echo "  /pobs/imu"
echo "  /pobs/sonar"
echo "  /model/pobs/pose"
echo "  /pobs/depth"
echo "  /pobs/pressure"
echo
echo "Leave this terminal open. Press Ctrl+C to stop both processes."

# Stop the other process if either one exits.
wait -n "$BRIDGE_PID" "$DEPTH_PID"
