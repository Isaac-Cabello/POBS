"""Observe the actual running lake pipeline across a soft reset."""

import importlib.util
import json
from pathlib import Path
import subprocess
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import FluidPressure, Imu, PointCloud2, Range
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


workspace = Path('/home/ros/ros2_ws')
spec = importlib.util.spec_from_file_location('reset_pobs', workspace / 'scripts/reset_pobs.py')
reset = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reset)
gazebo = reset.GazeboReset()
before_buoyancy = reset.buoyancy_components(gazebo.state())
assert before_buoyancy
rclpy.init()
node = Node('compatibility_live_observer')
latest, counts, minima = {}, {}, {}
phase = {'after_reset': False, 'cleared': set(), 'rewound': False, 'last_clock': None}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def receive(name, message):
    latest[name] = message
    counts[name] = counts.get(name, 0) + 1
    if name == 'clock':
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if phase['last_clock'] is not None and stamp < phase['last_clock']:
            phase['rewound'] = True
        phase['last_clock'] = stamp
    if name in ('truth_path', 'estimated_path', 'cloud'):
        size = message.width if name == 'cloud' else len(message.poses)
        if phase['after_reset']:
            minima[name] = min(minima.get(name, size), size)
            if size == 0:
                phase['cleared'].add(name)


subscriptions = []
for name, message_type, topic, profile in [
    ('truth_path', RosPath, '/visualization/ground_truth_path', qos),
    ('estimated_path', RosPath, '/visualization/estimated_path', qos),
    ('cloud', PointCloud2, '/pobs/mapping/points', qos),
    ('terrain', Marker, '/pobs/visualization/lake_terrain', qos),
    ('objects', MarkerArray, '/pobs/visualization/lake_obstacles', qos),
    ('tf', TFMessage, '/tf_static', qos),
    ('pose', PoseStamped, '/model/pobs/pose', qos_profile_sensor_data),
    ('odom', Odometry, '/state/odometry', qos_profile_sensor_data),
    ('depth', PointStamped, '/pobs/depth', qos_profile_sensor_data),
    ('pressure', FluidPressure, '/pobs/pressure', qos_profile_sensor_data),
    ('imu', Imu, '/pobs/imu', qos_profile_sensor_data),
    ('clock', Clock, '/clock', qos_profile_sensor_data),
    *[(name, Range, f'/pobs/sonar/sonar_{name}/range', qos_profile_sensor_data)
      for name in ('left', 'right', 'middle')],
    ('depth_sonar', Range, '/pobs/sonar/sonar_depth/range', qos_profile_sensor_data),
]:
    subscriptions.append(node.create_subscription(
        message_type, topic, lambda message, name=name: receive(name, message), profile))


def wait_until(predicate, seconds=35):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError({'counts': counts, 'minima': minima, 'phase': str(phase)})


try:
    wait_until(lambda: all(name in latest for name in
                          ('truth_path', 'estimated_path', 'cloud', 'clock'))
               and len(latest['truth_path'].poses) >= 3 and latest['cloud'].width > 0)
    phase['after_reset'] = True
    reset_process = subprocess.Popen(
        ['python3', str(workspace / 'scripts/reset_pobs.py')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    wait_until(lambda: reset_process.poll() is not None, 20)
    reset_output = reset_process.communicate()[0]
    assert reset_process.returncode == 0, reset_output
    required = {'truth_path', 'estimated_path', 'cloud'}
    wait_until(lambda: required <= phase['cleared'] and phase['rewound']
               and all(name in latest for name in ('left', 'right', 'middle', 'depth_sonar'))
               and latest['cloud'].width >= 20
               and len(latest['truth_path'].poses) >= 10
               and len(latest['estimated_path'].poses) >= 10
               and counts.get('terrain', 0) >= 3 and counts.get('objects', 0) >= 3)
    assert before_buoyancy == reset.buoyancy_components(gazebo.state())
    assert latest['truth_path'].header.frame_id == latest['estimated_path'].header.frame_id == 'lake'
    assert latest['cloud'].header.frame_id == 'lake'
    assert len(latest['objects'].markers) == 4
    assert latest['depth'].point.z >= 0 and latest['odom'].pose.pose.position.z <= 0
    report = {
        'passed': True, 'message_counts': counts,
        'reset_cleared': sorted(phase['cleared']),
        'clock_rewound': phase['rewound'], 'buoyant_links_preserved': len(before_buoyancy),
        'truth_trail_points': len(latest['truth_path'].poses),
        'estimated_trail_points': len(latest['estimated_path'].poses),
        'accumulated_sonar_points': latest['cloud'].width,
        'rviz_objects': len(latest['objects'].markers),
        'latest_ranges': {name: latest[name].range for name in
                          ('left', 'right', 'middle', 'depth_sonar')},
    }
    Path('/tmp/pobs-live-compatibility.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
finally:
    node.destroy_node()
    rclpy.try_shutdown()
