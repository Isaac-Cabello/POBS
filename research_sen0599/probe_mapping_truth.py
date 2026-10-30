"""Probe actual Gazebo PosePublisher metadata in an isolated static world."""
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

os.environ['GZ_PARTITION'] = 'pobs_mapping_truth_probe'
root = ET.fromstring('<sdf version="1.10"><world name="truth_probe">'
                     '<plugin filename="gz-sim-physics-system" '
                     'name="gz::sim::systems::Physics"/>'
                     '</world></sdf>')
model = ET.parse('/repo/ros2_ws/src/pobs_gazebo/models/pobs/model.sdf').getroot().find('model')
model.find('static').text = 'true'
for plugin in list(model.findall('plugin')):
    if 'PosePublisher' not in plugin.get('name', ''):
        model.remove(plugin)
root.find('world').append(model)
ET.ElementTree(root).write('/tmp/truth_probe.sdf')
log = open('/tmp/truth_probe.log', 'w+')
processes = []
rclpy.init()
node = Node('truth_metadata_probe')
messages = []
subscription = node.create_subscription(
    PoseStamped, '/model/pobs/pose', messages.append, qos_profile_sensor_data)
try:
    processes.append(subprocess.Popen(
        ['gz', 'sim', '-s', '-r', '/tmp/truth_probe.sdf'], stdout=log, stderr=log))
    processes.append(subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/model/pobs/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
        stdout=log, stderr=log))
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline and len(messages) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert messages, Path('/tmp/truth_probe.log').read_text()
    for msg in messages:
        print(repr(msg))
finally:
    for process in reversed(processes):
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    node.destroy_node()
    rclpy.try_shutdown()
