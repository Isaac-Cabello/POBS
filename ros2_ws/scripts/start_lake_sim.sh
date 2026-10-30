#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/lyrical/setup.bash

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/ros2_ws"
PACKAGE_DIR="$WORKSPACE_DIR/src/pobs_gazebo"

WORLD="$PACKAGE_DIR/worlds/lake.sdf"
BRIDGE_SCRIPT="$SCRIPT_DIR/pobs"

for file in "$WORLD" "$BRIDGE_SCRIPT"; do
  if [[ ! -f "$file" ]]; then
    echo "Required file not found: $file" >&2
    exit 1
  fi
done

export GZ_SIM_RESOURCE_PATH="$PACKAGE_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

GAZEBO_PID=""
BRIDGE_PID=""

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "Stopping POBS simulation and sensor processes..."

  # bridge_pobs.sh handles cleanup of its bridge and depth children.
  if [[ -n "$BRIDGE_PID" ]]; then
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
  fi

  if [[ -n "$GAZEBO_PID" ]]; then
    kill -TERM "$GAZEBO_PID" 2>/dev/null || true
  fi

  wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting lake and POBS..."
gz sim -r "$WORLD" &
GAZEBO_PID=$!

echo "Starting all four sonars, bridges and depth/pressure publisher..."
bash "$BRIDGE_SCRIPT" sensors source:=simulation &
BRIDGE_PID=$!

echo
echo "POBS simulation is starting."
echo "Gazebo and sensor processes share this terminal."
echo "Press Ctrl+C here to stop everything."

# If either launcher exits, clean up the other one.
wait -n "$GAZEBO_PID" "$BRIDGE_PID"
