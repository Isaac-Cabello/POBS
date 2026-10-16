"""Check pairing, bounded paths, and reset behavior using real ROS messages."""

from copy import deepcopy
from pathlib import Path as FilePath
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray

from pobs_gazebo.trajectory_comparison import TrajectoryComparison


@pytest.fixture
def node():
    """Create a fresh node for each test."""
    rclpy.init()
    comparison = TrajectoryComparison()
    yield comparison
    comparison.destroy_node()
    rclpy.shutdown()


def samples(second):
    """Create a matched pair in the lake frame."""
    estimate = Odometry()
    estimate.header.frame_id = 'lake'
    estimate.header.stamp.sec = second
    estimate.pose.pose.position.x = float(second)
    estimate.pose.pose.orientation.w = 1.0
    truth = PoseStamped()
    truth.header = deepcopy(estimate.header)
    truth.pose = deepcopy(estimate.pose.pose)
    return estimate, truth


def add_pair(node, second):
    """Deliver each half of a matched pair through its callback."""
    estimate, truth = samples(second)
    node.estimated_odometry_callback(estimate)
    node.ground_truth_odometry_callback(truth)


def test_pairing_and_path_limit(node):
    """Publish valid stamped paths, enforce the cap, and consume each pair."""
    node.max_path_points = 2
    for second in range(1, 5):
        add_pair(node, second)
    for path in (node.estimated_path, node.ground_truth_path):
        assert path.header.frame_id == 'lake'
        assert [pose.header.stamp.sec for pose in path.poses] == [3, 4]
        assert all(isinstance(pose, PoseStamped) for pose in path.poses)
    assert node.latest_estimated is None
    assert node.latest_truth is None


def test_unmatched_samples_wait(node):
    """Keep the latest messages until a pair falls within tolerance."""
    estimate, _ = samples(1)
    _, truth = samples(2)
    node.estimated_odometry_callback(estimate)
    node.ground_truth_odometry_callback(truth)
    assert not node.estimated_path.poses
    estimate, _ = samples(2)
    node.estimated_odometry_callback(estimate)
    assert len(node.estimated_path.poses) == 1


def test_clock_reset_clears_history_and_pending_samples(node):
    """A backward clock jump clears history and permits a fresh run."""
    add_pair(node, 10)
    clock = Clock()
    clock.clock.sec = 10
    node.clock_callback(clock)
    estimate, _ = samples(11)
    node.estimated_odometry_callback(estimate)
    clock.clock.sec = 0
    node.clock_callback(clock)
    assert not node.estimated_path.poses
    assert not node.ground_truth_path.poses
    assert node.latest_estimated is node.latest_truth is None
    add_pair(node, 1)
    assert [pose.header.stamp.sec for pose in node.estimated_path.poses] == [1]


@pytest.mark.parametrize('stream', ['estimate', 'truth'])
def test_input_timestamp_reset_without_clock(node, stream):
    """Backward input timestamps also clear trails without a clock publisher."""
    add_pair(node, 10)
    estimate, truth = samples(1)
    if stream == 'estimate':
        node.estimated_odometry_callback(estimate)
        node.ground_truth_odometry_callback(truth)
    else:
        node.ground_truth_odometry_callback(truth)
        node.estimated_odometry_callback(estimate)
    assert [pose.header.stamp.sec for pose in node.estimated_path.poses] == [1]
    assert [pose.header.stamp.sec for pose in node.ground_truth_path.poses] == [1]


def test_reset_publishes_empty_paths_to_subscribers(node):
    """RViz-style subscribers receive empty paths and retain the frame."""
    observer = Node('trajectory_reset_observer')
    received = {'estimate': [], 'truth': []}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        observer.create_subscription(
            Path, '/visualization/estimated_path',
            received['estimate'].append, qos),
        observer.create_subscription(
            Path, '/visualization/ground_truth_path',
            received['truth'].append, qos),
    ]

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert predicate()

    try:
        add_pair(node, 10)
        wait_for(lambda: all(messages for messages in received.values()))
        assert all(messages[-1].poses for messages in received.values())
        node.reset_comparison()
        wait_for(lambda: all(not messages[-1].poses
                             for messages in received.values()))
        assert all(messages[-1].header.frame_id == 'lake'
                   for messages in received.values())
    finally:
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        observer.destroy_node()


def test_lake_available_without_estimator(node):
    """A late subscriber gets both static meshes without any pose input."""
    observer = Node('lake_marker_observer')
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = observer.create_subscription(
        MarkerArray, '/visualization/lake', received.append, qos)
    try:
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert received
        assert len(received[-1].markers) == 2
        package_share = FilePath(get_package_share_directory('pobs_gazebo'))
        for marker in received[-1].markers:
            assert marker.header.frame_id == 'lake'
            assert marker.header.stamp.sec == marker.header.stamp.nanosec == 0
            assert marker.type == Marker.MESH_RESOURCE
            assert marker.pose.orientation.w == 1.0
            assert marker.scale.x == marker.scale.y == marker.scale.z == 1.0
            relative = marker.mesh_resource.removeprefix('package://pobs_gazebo/')
            assert (package_share / relative).is_file()
        assert not node.estimated_path.poses
        assert not node.ground_truth_path.poses
    finally:
        observer.destroy_subscription(subscription)
        observer.destroy_node()
