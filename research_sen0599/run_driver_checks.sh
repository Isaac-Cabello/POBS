#!/usr/bin/env bash
set -euo pipefail
docker run --rm --entrypoint /bin/bash \
  -v /mnt/c/Users/kotha/pobs/ros2_ws/src/pobs_sonar:/source:ro \
  -v /mnt/c/Users/kotha/pobs/ros2_ws/scripts:/scripts:ro \
  -v /mnt/c/Users/kotha/pobs/ros2_ws/src/pobs_gazebo/models/pobs:/robot_model:ro \
  -v /mnt/c/Users/kotha/pobs/ros2_ws/src/pobs_state_estimator:/state_estimator:ro \
  -v /mnt/c/Users/kotha/pobs/research_sen0599/check_mounted_sonars.py:/check_mounted_sonars.py:ro \
  -v /mnt/c/Users/kotha/pobs/research_sen0599/container_driver_checks.sh:/checks.sh:ro \
  pobs-ros2-lyrical:latest /checks.sh
