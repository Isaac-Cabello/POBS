
#!/usr/bin/env bash
set -eo pipefail

# ============================================================
# POBS BRIDGE STARTUP SCRIPT
# ============================================================

source /opt/ros/lyrical/setup.bash

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# ------------------------------------------------------------
# FILE PATHS
# ------------------------------------------------------------

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

DEPTH_SCRIPT="$SCRIPT_DIR/pobs_depth_sensor.py"

BRIDGE_CONFIG="$HOME/ros2_ws/src/pobs_gazebo/config/bridge.yaml"

# ------------------------------------------------------------
# VALIDATE REQUIRED FILES
# ------------------------------------------------------------

if [[ ! -f "$DEPTH_SCRIPT" ]]; then
    echo "ERROR: Missing depth script: $DEPTH_SCRIPT" >&2
    exit 1
fi

if [[ ! -f "$BRIDGE_CONFIG" ]]; then
    echo "ERROR: Missing bridge configuration: $BRIDGE_CONFIG" >&2
    exit 1
fi

# ------------------------------------------------------------
# PROCESS MANAGEMENT
# ------------------------------------------------------------

BRIDGE_PID=""
DEPTH_PID=""

cleanup() {
    trap - EXIT INT TERM

    echo
    echo "Stopping POBS bridge processes..."

    for pid in "$BRIDGE_PID" "$DEPTH_PID"; do
        if [[ -n "$pid" ]]; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    wait 2>/dev/null || true

    echo "POBS bridge processes stopped."
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ------------------------------------------------------------
# START ROS 2 / GAZEBO BRIDGES
# ------------------------------------------------------------

echo "Starting POBS ROS 2 / Gazebo bridges..."
echo "Using configuration: $BRIDGE_CONFIG"

ros2 run ros_gz_bridge parameter_bridge \
    --ros-args \
    -p config_file:="$BRIDGE_CONFIG" &

BRIDGE_PID=$!

# ------------------------------------------------------------
# START DEPTH AND PRESSURE PUBLISHER
# ------------------------------------------------------------

echo "Starting POBS depth and pressure publisher..."

python3 "$DEPTH_SCRIPT" &

DEPTH_PID=$!

# ------------------------------------------------------------
# STARTUP INFORMATION
# ------------------------------------------------------------

echo
echo "========================================"
echo "POBS BRIDGE SYSTEM STARTED"
echo "========================================"

echo
echo "Gazebo -> ROS 2:"
echo "  /clock"
echo "  /pobs/imu"
echo "  /pobs/sonar"
echo "  /model/pobs/pose"
echo
echo "ROS 2 -> Gazebo:"
echo "  /cmd_vel"
echo "  /model/pobs/joint/thruster_0_joint/cmd_thrust"
echo "  /model/pobs/joint/thruster_1_joint/cmd_thrust"
echo "  /model/pobs/joint/thruster_2_joint/cmd_thrust"
echo "  /model/pobs/joint/thruster_3_joint/cmd_thrust"
echo "  /model/pobs/joint/thruster_4_joint/cmd_thrust"
echo "  /model/pobs/joint/thruster_5_joint/cmd_thrust"
echo
echo "Additional ROS 2 publishers:"
echo "  /pobs/depth"
echo "  /pobs/pressure"
echo
echo "Leave this terminal open."
echo "Press Ctrl+C to stop both processes."
echo

# ------------------------------------------------------------
# WAIT FOR PROCESSES
# ------------------------------------------------------------

# Exit and clean up if either process stops.
wait -n "$BRIDGE_PID" "$DEPTH_PID"
