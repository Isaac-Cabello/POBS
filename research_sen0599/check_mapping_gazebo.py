"""Validate full mounted sonar-to-cloud pipeline against Gazebo floor and wall."""
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

os.environ['GZ_PARTITION'] = 'pobs_mapping_geometry_test'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
root = ET.fromstring('''<sdf version="1.10"><world name="mapping_test">
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
<render_engine>ogre2</render_engine></plugin>
<model name="floor"><static>true</static><link name="floor">
<pose>0 0 -2.05 0 0 0</pose>
<collision name="floor"><geometry><box><size>20 20 0.1</size></box></geometry></collision>
<visual name="floor"><geometry><box><size>20 20 0.1</size></box></geometry></visual>
</link></model>
<model name="wall"><static>true</static><link name="wall">
<pose>3.05 0 0 0 0 0</pose>
<collision name="wall"><geometry><box><size>0.1 20 10</size></box></geometry></collision>
<visual name="wall"><geometry><box><size>0.1 20 10</size></box></geometry></visual>
</link></model></world></sdf>''')
model_path = '/repo/ros2_ws/src/pobs_gazebo/models/pobs/model.sdf'
model = ET.parse(model_path).getroot().find('model')
model.find('static').text = 'true'
for plugin in list(model.findall('plugin')):
    if 'PosePublisher' not in plugin.get('name', ''):
        model.remove(plugin)
root.find('world').append(model)
ET.ElementTree(root).write('/tmp/mapping_geometry.sdf')
log = open('/tmp/mapping_geometry.log', 'w+')
processes = []
rclpy.init()
node = Node('mapping_gazebo_check')
clouds = []
subscription = node.create_subscription(
    PointCloud2, '/pobs/mapping/points', clouds.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
    for args in (
        ['gz', 'sim', '-s', '-r', '--headless-rendering', '/tmp/mapping_geometry.sdf'],
        ['ros2', 'launch', 'pobs_sonar', 'sonars.launch.py', 'source:=simulation'],
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/model/pobs/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ['ros2', 'run', 'pobs_mapping', 'point_cloud_mapper', '--ros-args',
         '-p', 'model_path:=' + model_path, '-p', 'world_frame:=mapping_test',
         '-p', 'use_sim_time:=true'],
    ):
        processes.append(subprocess.Popen(args, start_new_session=True,
                                          stdout=log, stderr=log))
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        assert all(process.poll() is None for process in processes), 'Early exit'
        rclpy.spin_once(node, timeout_sec=0.1)
        if clouds and clouds[-1].width >= 100:
            break
    assert clouds and clouds[-1].width >= 100, 'No accumulated full cloud'
    points = np.array([list(row) for row in point_cloud2.read_points(
        clouds[-1], field_names=('x', 'y', 'z'))])
    floor = np.abs(points[:, 2] + 2.) < 0.1
    wall = np.abs(points[:, 0] - 3.) < 0.1
    assert np.all(floor | wall), points[~(floor | wall)]
    assert floor.sum() > 40 and wall.sum() > 10, (floor.sum(), wall.sum())
    print(f'PASS: Gazebo truth + four mounted sensors produce {len(points)} '
          f'world points on known floor ({floor.sum()}) and wall ({wall.sum()})')
except BaseException:
    log.flush()
    print(Path('/tmp/mapping_geometry.log').read_text()[-24000:])
    raise
finally:
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()
    log.close()
