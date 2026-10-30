#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
cd /home/ros/ros2_ws
python3 scripts/sync_robot_variants.py
colcon build --symlink-install --packages-up-to pobs_gazebo pobs_sonar pobs_stabilizer
source install/setup.bash
export ROS_DOMAIN_ID=92
mkdir -p /tmp/pobs-host
if [[ ! -e /tmp/pobs-host/ros2_ws ]]; then
  ln -s /home/ros/ros2_ws /tmp/pobs-host/ros2_ws
fi
python3 -m pytest -q \
  -o cache_dir=/tmp/pobs-audit-pytest-cache \
  src/pobs_gazebo/test/test_trajectory_comparison.py \
  src/pobs_gazebo/test/test_estimator_integration.py \
  src/pobs_gazebo/test/test_robot_compatibility.py \
  src/pobs_gazebo/test/test_reset_pobs.py \
  src/pobs_gazebo/test/test_lake_terrain.py \
  src/pobs_state_estimator/test/test_motion.py \
  src/pobs_mapping/test src/pobs_sonar/test src/pobs_visualization/test \
  /tmp/pobs-host/tests/test_host_pobs.py
