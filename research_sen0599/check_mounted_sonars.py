"""Headless integration test of the mounted model against known surfaces."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Range
from tf2_msgs.msg import TFMessage

os.environ['GZ_PARTITION'] = 'pobs_mount_geometry_test'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
root = ET.fromstring('''<sdf version="1.10"><world name="mount_test">
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
</link></model>
</world></sdf>''')
model = ET.parse('/robot_model/model.sdf').getroot().find('model')
model.find('static').text = 'true'
# Keep the exact model sensor geometry; isolate it from thruster dynamics.
for plugin in list(model.findall('plugin')):
    model.remove(plugin)
root.find('world').append(model)
ET.ElementTree(root).write('/tmp/sonar_mount_test.sdf')
log = open('/tmp/sonar_mount_test.log', 'w+')
processes = []
rclpy.init()
node = Node('mounted_sonar_check')
values = {}
frames = set()
subscriptions = []
for position in ('left', 'right', 'middle', 'depth'):
    subscriptions.append(node.create_subscription(
        Range, f'/pobs/sonar/sonar_{position}/range',
        lambda message, position=position: values.__setitem__(position, message),
        qos_profile_sensor_data))
subscriptions.append(node.create_subscription(
    TFMessage, '/tf_static',
    lambda message: frames.update(
        transform.child_frame_id for transform in message.transforms
        if transform.header.frame_id == 'base_link'),
    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
try:
    processes.append(subprocess.Popen(
        ['gz', 'sim', '-s', '-r', '--headless-rendering', '/tmp/sonar_mount_test.sdf'],
        start_new_session=True, stdout=log, stderr=log))
    processes.append(subprocess.Popen(
        ['pobs', 'sensors', 'source:=simulation', 'existing_sensors:=false'],
        start_new_session=True, stdout=log, stderr=log))
    required_frames = {f'sonar_{position}_link'
                       for position in ('left', 'right', 'middle', 'depth')}
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        assert all(process.poll() is None for process in processes), 'Early exit'
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(values) == 4 and required_frames <= frames:
            break
    assert len(values) == 4, f'Missing simulated distances: {values}'
    assert required_frames <= frames, frames
    expected = {'left': 1.98 * math.sqrt(2), 'right': 1.98 * math.sqrt(2),
                'middle': 3.0 - 0.44, 'depth': 2.0 - 0.08}
    for position, distance in expected.items():
        assert abs(values[position].range - distance) < 0.08, (
            position, values[position].range, distance)
        assert values[position].header.frame_id == f'sonar_{position}_link'
    print('PASS: mounted Gazebo sensors measure known floor and wall distances; '
          'all four mounting frames are published')
    print({position: round(message.range, 3) for position, message in values.items()})
except BaseException:
    log.flush()
    print(Path('/tmp/sonar_mount_test.log').read_text()[-12000:])
    raise
finally:
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()
    log.close()
