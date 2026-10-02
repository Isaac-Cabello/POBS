#!/usr/bin/env bash
set -e

mkdir -p /home/ros/ros2_ws/build
mkdir -p /home/ros/ros2_ws/install
mkdir -p /home/ros/ros2_ws/log

chown -R ros:ros \
  /home/ros/ros2_ws/build \
  /home/ros/ros2_ws/install \
  /home/ros/ros2_ws/log

exec gosu ros "$@"