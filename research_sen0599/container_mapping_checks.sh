#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=93
apt-get update -qq
apt-get install -y --no-install-recommends python3-serial python3-pytest
mkdir -p /tmp/mapping_ws/src
for package in pobs_mapping pobs_sonar pobs_visualization; do
  cp -r "/repo/ros2_ws/src/$package" /tmp/mapping_ws/src/
done
cp -r /repo/ros2_ws/scripts /tmp/mapping_ws/
cp -r /repo/ros2_ws/src/pobs_gazebo /tmp/mapping_ws/src/
cp -r /repo/ros2_ws/src/pobs_state_estimator /tmp/mapping_ws/src/
cd /tmp/mapping_ws
colcon build --packages-up-to pobs_mapping pobs_sonar pobs_visualization
source install/setup.bash
colcon test --packages-select pobs_mapping pobs_sonar pobs_visualization --event-handlers console_direct+
colcon test-result --verbose
python3 /repo/research_sen0599/check_mapping_ros.py
python3 /repo/research_sen0599/check_mapping_gazebo.py
ament_flake8 /repo/ros2_ws/src/pobs_mapping /repo/ros2_ws/src/pobs_visualization
python3 /repo/research_sen0599/check_mapping_cli.py
