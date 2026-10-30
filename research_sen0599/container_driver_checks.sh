#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
apt-get update -qq
apt-get install -y --no-install-recommends python3-serial python3-pytest
python3 -c 'import serial, pytest, rclpy'
mkdir -p /tmp/sonar_ws/src
cp -r /source /tmp/sonar_ws/src/pobs_sonar
cp -r /scripts /tmp/sonar_ws/scripts
mkdir -p /tmp/sonar_ws/src/pobs_gazebo/models
cp -r /robot_model /tmp/sonar_ws/src/pobs_gazebo/models/pobs
cp -r /state_estimator /tmp/sonar_ws/src/pobs_state_estimator
cp /scripts/pobs /usr/local/bin/pobs
chmod +x /usr/local/bin/pobs
export POBS_WORKSPACE=/tmp/sonar_ws
bash -n /scripts/pobs
pobs --help
cd /tmp/sonar_ws
colcon build --packages-select pobs_sonar
source install/setup.bash
colcon test --packages-select pobs_sonar --event-handlers console_direct+
colcon test-result --verbose
ros2 launch pobs_sonar sonars.launch.py --show-args
ament_flake8 src/pobs_sonar
gz sdf -k /robot_model/model.sdf
python3 - <<'PY'
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import FluidPressure

process = subprocess.Popen(
    ['pobs', 'sensors'],
    start_new_session=True)
rclpy.init()
node = Node('sonar_launch_smoke_test')
positions = ('left', 'right', 'middle', 'depth')
expected = {f'/pobs/sonar/sonar_{position}/range': ['sensor_msgs/msg/Range']
            for position in positions}
expected.update({
    '/pobs/imu': ['sensor_msgs/msg/Imu'],
    '/pobs/depth': ['geometry_msgs/msg/PointStamped'],
    '/pobs/pressure': ['sensor_msgs/msg/FluidPressure'],
    '/model/pobs/pose': ['geometry_msgs/msg/PoseStamped'],
})
try:
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        topics = dict(node.get_topic_names_and_types())
        if expected.keys() <= topics.keys():
            break
        assert process.poll() is None, 'Launch exited early'
        rclpy.spin_once(node, timeout_sec=0.1)
    assert expected.keys() <= topics.keys(), topics
    for topic in expected:
        assert topics[topic] == expected[topic]
    depth_values = []
    pressure_values = []
    depth_sub = node.create_subscription(
        PointStamped, '/pobs/depth', lambda msg: depth_values.append(msg.point.z), 10)
    pressure_sub = node.create_subscription(
        FluidPressure, '/pobs/pressure',
        lambda msg: pressure_values.append(msg.fluid_pressure), 10)
    pose_pub = node.create_publisher(PoseStamped, '/model/pobs/pose', 10)
    pose = PoseStamped()
    pose.pose.position.z = -2.0
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not (depth_values and pressure_values):
        pose_pub.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.1)
    assert depth_values and depth_values[-1] == 2.0, depth_values
    assert pressure_values and pressure_values[-1] == 120945.0, pressure_values
    for i, position in enumerate(positions):
        client = AsyncParameterClient(node, f'/pobs/sonar/sonar_{position}')
        assert client.wait_for_services(timeout_sec=10), position
        future = client.get_parameters(['port'])
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), position
        assert future.result().values[0].string_value == f'/dev/ttyUSB{i}'
    print('PASS: pobs sensors activates all existing and named sonar topics')
finally:
    node.destroy_node()
    rclpy.shutdown()
    process.send_signal(signal.SIGINT)
    process.wait(timeout=10)
PY
python3 /check_mounted_sonars.py
