#!/usr/bin/env bash
set -eo pipefail
docker run --rm --entrypoint /bin/bash \
  -v /mnt/c/Users/kotha/pobs:/repo:ro \
  pobs-ros2-lyrical:latest /repo/research_sen0599/container_mapping_checks.sh
