"""Check the mapping-only view, its TF root, late-joining markers, and RViz."""

import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '76'
os.environ['DISPLAY'] = ':93'
os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ.pop('WAYLAND_DISPLAY', None)

from geometry_msgs.msg import PoseStamped  # noqa: E402
from PIL import ImageGrab  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import PointCloud2, Range  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


processes = []
log = Path('/tmp/pobs-scene-check.log').open('w')
rclpy.init()
node = Node('scene_regression_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
terrain, water, clouds = [], [], []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions = []
try:
    processes.append(subprocess.Popen(
        ['Xvfb', ':93', '-screen', '0', '1280x960x24', '-ac'],
        start_new_session=True, stdout=log, stderr=log))
    time.sleep(0.5)
    processes.append(subprocess.Popen(
        ['ros2', 'launch', 'pobs_visualization', 'visualization.launch.py',
         'state_estimation:=false', 'point_cloud_mapping:=true', 'use_sim_time:=false'],
        start_new_session=True, stdout=log, stderr=log))
    # Subscribe after scene startup to exercise retained geometry, not just live messages.
    time.sleep(4)
    subscriptions.extend([
        node.create_subscription(Marker, '/pobs/visualization/lake_terrain', terrain.append, qos),
        node.create_subscription(Marker, '/pobs/visualization/lake_water', water.append, qos),
        node.create_subscription(PointCloud2, '/pobs/mapping/points', clouds.append, qos),
    ])
    pose_pub = node.create_publisher(PoseStamped, '/model/pobs/pose', 10)
    ranges = {name: node.create_publisher(Range, f'/pobs/sonar/sonar_{name}/range', 10)
              for name in ('left', 'right', 'middle', 'depth')}
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        assert all(process.poll() is None for process in processes), 'Process exited early'
        stamp = node.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id = 'lake'
        pose.header.stamp = stamp
        pose.pose.position.x = -12.0
        pose.pose.position.y = -8.0
        pose.pose.position.z = -3.0
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)
        for name, publisher in ranges.items():
            reading = Range()
            reading.header.frame_id = f'sonar_{name}_link'
            reading.header.stamp = stamp
            reading.min_range = 0.15
            reading.max_range = 30.0
            reading.range = 5.0
            publisher.publish(reading)
        rclpy.spin_once(node, timeout_sec=0.1)
        if (terrain and water and clouds and clouds[-1].width >= 20
                and buffer.can_transform('lake', 'pobs_reference_scene', rclpy.time.Time())):
            break
    assert terrain and len(terrain[-1].points) > 1000, 'No retained terrain'
    assert water and water[-1].points, 'No retained water'
    assert clouds and clouds[-1].width >= 20, 'No accumulated points'
    assert buffer.can_transform('lake', 'pobs_reference_scene', rclpy.time.Time()), \
        'Mapping-only launch did not register lake in TF'
    assert node.count_subscribers('/state/odometry') == 0, 'Unexpected estimator dependency'
    # RViz and this check both subscribe to the terrain with compatible QoS.
    assert node.count_subscribers('/pobs/visualization/lake_terrain') >= 2, \
        'RViz did not subscribe to the terrain display'
    time.sleep(2)
    ImageGrab.grab(xdisplay=':93').save('/tmp/pobs-scene-check.png')
    print(f'PASS: mapping-only TF root, {len(terrain[-1].points) // 3} terrain triangles, '
          f'retained water, {clouds[-1].width} map points, RViz terrain subscription')
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
    output = Path('/tmp/pobs-scene-check.log').read_text()
    print(output[-8000:])
    assert 'process has died' not in output and 'Traceback' not in output, output
