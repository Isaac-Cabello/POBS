"""Verify our depth and visualization adapters with the unchanged estimator."""

from copy import deepcopy
import importlib.util
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from pobs_state_estimator.state_estimator import StateEstimator
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from pobs_gazebo.trajectory_comparison import TrajectoryComparison


def test_depth_estimator_and_paths_use_compatible_messages():
    """Stamped depth reaches the estimator and produces paths with static TF."""
    script = FilePath(__file__).resolve().parents[3] / 'scripts/pobs_depth_sensor.py'
    spec = importlib.util.spec_from_file_location('depth_adapter_under_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    depth = module.Depth()
    estimator = StateEstimator()
    comparison = TrajectoryComparison()
    observer = Node('estimator_integration_observer')
    executor = SingleThreadedExecutor()
    nodes = [depth, estimator, comparison, observer]
    for node in nodes:
        executor.add_node(node)
    clock_pub = observer.create_publisher(Clock, '/clock', 10)
    paths = []
    transforms = []
    raw_odometry = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_sub = observer.create_subscription(
        Path, '/visualization/estimated_path', paths.append, qos)
    tf_sub = observer.create_subscription(
        TFMessage, '/tf_static', transforms.append, qos)
    odom_sub = observer.create_subscription(
        Odometry, '/state/odometry', raw_odometry.append, 10)

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            clock = Clock()
            clock.clock.sec = 10
            clock_pub.publish(clock)
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        wait_for(lambda: all(node.get_clock().now().nanoseconds == 10_000_000_000
                             for node in nodes))
        wait_for(lambda: depth.depth.get_subscription_count() >= 1)
        truth = PoseStamped()
        truth.header.frame_id = 'lake'
        truth.header.stamp.sec = 10
        truth.pose.orientation.w = 1.0
        truth.pose.position.x = -12.0
        truth.pose.position.y = -8.0
        truth.pose.position.z = -3.2
        original_header = deepcopy(truth.header)
        depth.update(truth)
        # The pressure publisher must not mutate the shared source header.
        assert truth.header == original_header
        imu = Imu()
        imu.header = deepcopy(truth.header)
        imu.orientation.w = 1.0
        estimator.imu_callback(imu)
        comparison.ground_truth_odometry_callback(truth)
        wait_for(lambda: estimator.depth_m is not None)
        assert estimator.depth_m == 3.2
        estimator.publish_odometry()
        wait_for(lambda: bool(paths) and bool(transforms) and bool(raw_odometry))
        path = paths[-1]
        assert path.header.frame_id == 'lake'
        assert path.poses[-1].header.frame_id == 'lake'
        assert path.header.stamp.sec == 10
        assert path.poses[-1].pose.position.z == -3.2
        assert comparison.ground_truth_path.header.frame_id == 'lake'
        transform = next(t for message in transforms for t in message.transforms
                         if t.child_frame_id == 'odom')
        assert transform.header.frame_id == 'lake'
        assert transform.transform.translation.x == -12.0
        assert transform.transform.translation.y == -8.0
        assert transform.transform.translation.z == 0.0
        # The fixed offset applies to the visualization only.
        assert path.poses[-1].pose.position.x == -12.0
        assert path.poses[-1].pose.position.y == -8.0
        assert raw_odometry[-1].header.frame_id == 'odom'
        assert raw_odometry[-1].pose.pose.position.x == 0.0
        assert raw_odometry[-1].pose.pose.position.y == 0.0
    finally:
        observer.destroy_subscription(path_sub)
        observer.destroy_subscription(tf_sub)
        observer.destroy_subscription(odom_sub)
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
