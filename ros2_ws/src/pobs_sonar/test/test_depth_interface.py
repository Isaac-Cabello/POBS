"""Verify the depth publisher and existing state estimator work together."""

import importlib.util
from pathlib import Path
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stamped_depth_preserves_depth_pressure_and_odometry():
    workspace = Path(__file__).parents[3]
    depth_module = load_module(
        'depth_adapter', workspace / 'scripts' / 'pobs_depth_sensor.py')
    estimator_module = load_module(
        'depth_estimator', workspace / 'src' / 'pobs_state_estimator'
        / 'pobs_state_estimator' / 'state_estimator.py')
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        depth = depth_module.Depth()
        estimator = estimator_module.StateEstimator()
        observer = Node('depth_interface_test')
        nodes.extend((depth, estimator, observer))
        for node in nodes:
            executor.add_node(node)
        received = {'depth': [], 'pressure': [], 'odometry': []}
        subscriptions = [
            observer.create_subscription(
                msg_type, topic,
                lambda msg, key=key: received[key].append(msg), 10)
            for key, msg_type, topic in (
                ('depth', PointStamped, '/pobs/depth'),
                ('pressure', FluidPressure, '/pobs/pressure'),
                ('odometry', Odometry, '/state/odometry'))
        ]
        pose_pub = observer.create_publisher(PoseStamped, '/model/pobs/pose', 10)
        imu_pub = observer.create_publisher(Imu, '/pobs/imu', 10)
        pose = PoseStamped()
        pose.header.stamp.sec = 10
        pose.header.frame_id = 'lake'
        pose.pose.position.z = -2.0
        imu = Imu()
        imu.header.stamp.sec = 10
        imu.orientation.w = 1.0
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not all(received.values()):
            pose_pub.publish(pose)
            imu_pub.publish(imu)
            executor.spin_once(timeout_sec=0.01)
        assert all(received.values()), received
        assert len(subscriptions) == 3
        assert received['depth'][-1].point.z == 2.0
        assert received['depth'][-1].header.stamp == pose.header.stamp
        assert received['depth'][-1].header.frame_id == 'lake'
        assert received['pressure'][-1].fluid_pressure == pytest.approx(120945.0)
        assert received['odometry'][-1].pose.pose.position.z == -2.0
        assert pose.header.frame_id == 'lake'
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()
