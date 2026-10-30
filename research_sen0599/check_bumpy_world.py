"""Check four real Gazebo sonar returns against the generated scene geometry."""

import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

os.environ['ROS_DOMAIN_ID'] = '81'
os.environ['GZ_PARTITION'] = f'pobs_bumpy_check_{os.getpid()}'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['POBS_WORKSPACE'] = '/home/ros/ros2_ws'
os.environ['HOME'] = '/home/ros'

import numpy as np  # noqa: E402
from pobs_sonar.mounting import SONAR_MOUNTS  # noqa: E402
from pobs_visualization.scene import lake_geometry, pose_matrix  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile  # noqa: E402
from sensor_msgs.msg import PointCloud2, Range  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402


def ray_distance(origin, direction, triangles):
    """Find the nearest triangle intersection using an independent ray test."""
    first = triangles[:, 1] - triangles[:, 0]
    second = triangles[:, 2] - triangles[:, 0]
    cross = np.cross(np.broadcast_to(direction, second.shape), second)
    determinant = np.sum(first * cross, axis=1)
    valid = abs(determinant) > 1e-10
    inverse = np.zeros_like(determinant)
    inverse[valid] = 1 / determinant[valid]
    offset = origin - triangles[:, 0]
    u = np.sum(offset * cross, axis=1) * inverse
    q = np.cross(offset, first)
    v = np.sum(direction * q, axis=1) * inverse
    distance = np.sum(second * q, axis=1) * inverse
    valid &= (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (distance >= 0.05)
    return float(distance[valid].min())


package = Path('/home/ros/ros2_ws/src/pobs_gazebo')
os.environ['GZ_SIM_RESOURCE_PATH'] = str(package / 'models')
cad = os.environ.get('POBS_TEST_CAD') == '1'
world_path = package / ('worlds/lake_cad.sdf' if cad else 'worlds/lake.sdf')
root = ET.parse(world_path).getroot()
world = root.find('world')
gui = world.find('gui')
if gui is not None:
    world.remove(gui)
include = next(item for item in world.findall('include') if item.findtext('name') == 'pobs')
spawn = pose_matrix(include.find('pose'))
variant = 'pobs_cad' if cad else 'pobs'
model = ET.parse(package / f'models/{variant}/model.sdf').getroot().find('model')
# The included world deliberately names either visual variant `pobs` so bridges
# and motor topics keep the same interface.
model.set('name', 'pobs')
model.find('static').text = 'true'
pose = ET.SubElement(model, 'pose')
pose.text = include.findtext('pose')
for plugin in list(model.findall('plugin')):
    if 'PosePublisher' not in plugin.get('name', ''):
        model.remove(plugin)
world.remove(include)
world.append(model)
ET.ElementTree(root).write('/tmp/pobs_bumpy_check.sdf')
geometry = lake_geometry(world_path)
triangles = np.vstack([points for name, points in geometry.items()
                       if name != 'water_surface']).reshape(-1, 3, 3)
expected = {}
endpoints = []
for name, mount in SONAR_MOUNTS.items():
    element = ET.fromstring('<pose>' + ' '.join(map(str, mount)) + '</pose>')
    transform = spawn @ pose_matrix(element)
    origin, direction = transform[:3, 3], transform[:3, 0]
    expected[name] = ray_distance(origin, direction, triangles)
    assert expected[name] < (30 if name == 'depth' else 6), expected
    endpoints.append(origin + direction * expected[name])
print('Expected closest ranges:', expected, flush=True)

processes = []
log_path = Path('/tmp/pobs-bumpy-check.log')
log = log_path.open('w')
rclpy.init()
node = Node('bumpy_world_check')
readings = {name: [] for name in SONAR_MOUNTS}
clouds, markers = [], []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions = [
    node.create_subscription(PointCloud2, '/pobs/mapping/points', clouds.append, qos),
    node.create_subscription(MarkerArray, '/pobs/visualization/lake_obstacles', markers.append, qos),
]
for name in readings:
    subscriptions.append(node.create_subscription(
        Range, f'/pobs/sonar/sonar_{name}/range',
        lambda message, name=name: readings[name].append(message.range), qos_profile_sensor_data))
try:
    for command in (
        ['gz', 'sim', '-s', '-r', '--headless-rendering', '/tmp/pobs_bumpy_check.sdf'],
        ['ros2', 'launch', 'pobs_sonar', 'sensors.launch.py', 'source:=simulation'],
        ['ros2', 'launch', 'pobs_visualization', 'visualization.launch.py',
         'state_estimation:=false', 'point_cloud_mapping:=true', 'rviz:=false'],
    ):
        processes.append(subprocess.Popen(command, start_new_session=True,
                                          stdout=log, stderr=log))
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        assert all(process.poll() is None for process in processes), 'Early process exit'
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(len(values) >= 10 for values in readings.values()) \
                and clouds and clouds[-1].width >= 100 and markers:
            break
    for name, values in readings.items():
        assert len(values) >= 10, (name, len(values))
        assert abs(float(np.median(values)) - expected[name]) < 0.08, \
            (name, np.median(values), expected[name])
    assert clouds and clouds[-1].width >= 100, 'No accumulated map'
    points = point_cloud2.read_points_numpy(clouds[-1], field_names=('x', 'y', 'z'))
    distances = np.linalg.norm(points[:, None, :] - np.asarray(endpoints)[None, :, :], axis=2)
    assert distances.min(axis=1).max() < 0.12, 'Map points miss the true target surfaces'
    assert markers and len(markers[-1].markers) == 4, 'Missing RViz reference objects'
    print(f'PASS: {clouds[-1].width} actual sonar points on terrain/boulders/wall; '
          'all four ranges agree with independent mesh intersections; four RViz objects.', flush=True)
    if os.environ.get('POBS_PREVIEW') == '1':
        from PIL import ImageGrab
        from pobs_visualization.configuration import make_config
        import yaml

        display_env = dict(os.environ, DISPLAY=':95', QT_QPA_PLATFORM='xcb')
        display_env.pop('WAYLAND_DISPLAY', None)
        processes.append(subprocess.Popen(
            ['Xvfb', ':95', '-screen', '0', '1280x960x24', '-ac'],
            env=display_env, start_new_session=True, stdout=log, stderr=log))
        time.sleep(0.5)
        config = make_config(False, True)
        view = config['Visualization Manager']['Views']['Current']
        view['Distance'] = 24
        view['Focal Point'] = {'X': -11, 'Y': -7.5, 'Z': -7.5}
        view['Yaw'] = 2.4
        config['Visualization Manager']['Displays'][0]['Enabled'] = False
        config_path = '/tmp/pobs-bumpy-preview.rviz'
        Path(config_path).write_text(yaml.safe_dump(config, sort_keys=False))
        processes.append(subprocess.Popen(
            ['rviz2', '-d', config_path, '--ros-args', '-p', 'use_sim_time:=true'],
            env=display_env, start_new_session=True, stdout=log, stderr=log))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_subscribers('/pobs/visualization/lake_obstacles') >= 2:
                break
        assert node.count_subscribers('/pobs/visualization/lake_obstacles') >= 2
        time.sleep(3)
        ImageGrab.grab(xdisplay=':95').save('/tmp/pobs-bumpy-preview.png')
        print('Saved rendered RViz test-area preview: /tmp/pobs-bumpy-preview.png', flush=True)
finally:
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
    node.destroy_node()
    rclpy.try_shutdown()
    log.close()
    print(log_path.read_text()[-14000:])
