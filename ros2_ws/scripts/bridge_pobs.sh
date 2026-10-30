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
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
}

trap cleanup EXIT
# A requested launcher shutdown is normal; unexpected child failures still
# propagate through wait -n below.
trap 'exit 0' INT
trap 'exit 0' TERM

echo "Starting POBS sensor and pose bridges..."

# Isolate children from the launcher's signal group. Cleanup delivers one
# termination signal to each executable instead of racing SIGINT against SIGTERM.
BRIDGE_EXECUTABLE="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
setsid "$BRIDGE_EXECUTABLE" \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/pobs/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/model/pobs/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' &
BRIDGE_PID=$!

echo "Starting POBS depth and pressure publisher..."

setsid python3 "$DEPTH_SCRIPT" &
DEPTH_PID=$!

echo "POBS topics:"
echo "  /clock"
echo "  /pobs/imu"
echo "  /model/pobs/pose"
echo "  /pobs/depth (PointStamped: positive depth in point.z)"
echo "  /pobs/pressure"
echo
echo "Leave this terminal open. Press Ctrl+C to stop both processes."

# Stop the other process if either one exits.
wait -n "$BRIDGE_PID" "$DEPTH_PID"
