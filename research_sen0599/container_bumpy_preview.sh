#!/usr/bin/env bash
set -eo pipefail
export HOME=/home/ros
apt-get update -qq
apt-get install -y --no-install-recommends xvfb python3-serial
source /opt/ros/lyrical/setup.bash
mkdir -p /home/ros/ros2_ws/src/pobs_gazebo
for package in pobs_mapping pobs_sonar pobs_visualization; do
  cp -r "/repo/ros2_ws/src/$package" /home/ros/ros2_ws/src/
done
for directory in models worlds; do
  if [[ ! -e "/home/ros/ros2_ws/src/pobs_gazebo/$directory" ]]; then
    ln -s "/repo/ros2_ws/src/pobs_gazebo/$directory" "/home/ros/ros2_ws/src/pobs_gazebo/$directory"
  fi
done
if [[ ! -e /home/ros/ros2_ws/scripts ]]; then
  ln -s /repo/ros2_ws/scripts /home/ros/ros2_ws/scripts
fi
cd /home/ros/ros2_ws
colcon build --packages-select pobs_mapping pobs_sonar pobs_visualization
source install/setup.bash
export POBS_PREVIEW=1
python3 /repo/research_sen0599/check_bumpy_world.py
