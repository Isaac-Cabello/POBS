"""Inspect the real running mapping pipeline without injecting or clearing data."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


rclpy.init()
node = Node('live_mapping_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
samples = {name: [] for name in ('left', 'right', 'middle', 'depth')}
clouds = []
subscriptions = [node.create_subscription(
    PointCloud2, '/pobs/mapping/points', clouds.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
for name in samples:
    subscriptions.append(node.create_subscription(
        Range, f'/pobs/sonar/sonar_{name}/range',
        lambda message, name=name: samples[name].append(message.range),
        qos_profile_sensor_data))
try:
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    points = point_cloud2.read_points_numpy(clouds[-1], field_names=('x', 'y', 'z')) \
        if clouds and clouds[-1].width else None
    result = {
        'tf_lake_registered': buffer.can_transform(
            'lake', 'pobs_reference_scene', rclpy.time.Time()),
        'cloud_frame': clouds[-1].header.frame_id if clouds else None,
        'first_point_count': clouds[0].width if clouds else None,
        'last_point_count': clouds[-1].width if clouds else None,
        'points_added_during_check': clouds[-1].width - clouds[0].width if clouds else None,
        'cloud_updates': len(clouds),
        'bounds_min_xyz': points.min(axis=0).tolist() if points is not None else None,
        'bounds_max_xyz': points.max(axis=0).tolist() if points is not None else None,
        'sensors': {name: {
            'messages': len(values),
            'finite_hits': sum(math.isfinite(value) for value in values),
            'last_range_m': values[-1] if values and math.isfinite(values[-1]) else 'no return',
        } for name, values in samples.items()},
    }
    print(json.dumps(result, indent=2))
finally:
    node.destroy_node()
    rclpy.try_shutdown()
