"""Accumulate every accepted sonar hit in a fixed world coordinate frame."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import os
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
from pobs_mapping.core import PointAccumulator, PoseBuffer
from pobs_mapping.geometry import load_sensor_transforms
import rclpy
from rclpy.clock import Clock as RosClock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


POSITIONS = ('left', 'right', 'middle', 'depth')


def stamp_ns(stamp):
    """Convert an acquisition timestamp into integer nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass
class PendingReading:
    """A valid reading waiting for bracketing ground-truth poses."""

    sensor: str
    distance: float
    min_range: float
    max_range: float
    stamp: int
    received_at: float


class PointCloudMapper(Node):
    """Project Range readings using SDF mounts and timestamped true model pose.

    No subscription to state-estimator odometry exists. Every accepted point is
    kept in RAM until explicitly cleared, a simulation reset, or process exit.
    """

    def __init__(self, **kwargs):
        super().__init__('point_cloud_mapper', **kwargs)
        workspace = os.environ.get('POBS_WORKSPACE', os.path.expanduser('~/ros2_ws'))
        defaults = {
            'model_path': os.path.join(
                workspace, 'src', 'pobs_gazebo', 'models', 'pobs', 'model.sdf'),
            'world_frame': 'lake',
            'pose_topic': '/model/pobs/pose',
            'publish_rate_hz': 5.0,
            'pose_history_s': 10.0,
            'max_pose_gap_s': 0.5,
            'pending_timeout_s': 1.0,
            'pose_reset_threshold_s': 0.5,
            'max_pending_readings': 1000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.world_frame = self.get_parameter('world_frame').value
        if not self.world_frame:
            raise ValueError('world_frame must be nonempty')
        history = self.positive_parameter('pose_history_s')
        self.max_pose_gap_ns = int(self.positive_parameter('max_pose_gap_s') * 1e9)
        self.pending_timeout = self.positive_parameter('pending_timeout_s')
        self.reset_threshold_ns = int(
            self.positive_parameter('pose_reset_threshold_s') * 1e9)
        rate = self.positive_parameter('publish_rate_hz')
        self.max_pending = int(self.get_parameter('max_pending_readings').value)
        if self.max_pending <= 0:
            raise ValueError('max_pending_readings must be positive')
        self.sensor_names = {position: f'sonar_{position}' for position in POSITIONS}
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        transforms = load_sensor_transforms(model_path, self.sensor_names.values())
        self.accumulator = PointAccumulator(transforms)
        self.poses = PoseBuffer(max_age_ns=int(history * 1e9))
        self.pending = deque()
        self.last_clock_ns = None
        self.latest_point_stamp = 0
        self.dirty = True

        # A late RViz subscriber receives the latest complete accumulated map.
        cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pobs/mapping/points', cloud_qos)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/pobs/mapping/ground_truth_pose', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self.pose_callback, qos_profile_sensor_data)
        self.range_subscriptions = [
            self.create_subscription(
                Range, f'/pobs/sonar/sonar_{position}/range',
                lambda message, position=position: self.range_callback(position, message),
                qos_profile_sensor_data)
            for position in POSITIONS
        ]
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile_sensor_data)
        self.clear_service = self.create_service(
            Trigger, '/pobs/mapping/clear', self.clear_callback)
        # Continue publishing empty maps and expiring pending readings when the
        # simulation is paused or its clock has not started yet.
        self.timer = self.create_timer(
            1.0 / rate, self.tick, clock=RosClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Loaded four SDF sensor transforms from {model_path}; '
            f'accumulating all accepted points in {self.world_frame}')

    def positive_parameter(self, name):
        """Reject invalid timing settings at startup."""
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def warn(self, message):
        """Keep bad sensor streams from flooding the console."""
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def clock_callback(self, message):
        """Clear points and pose history when simulation time goes backward."""
        current = stamp_ns(message.clock)
        if self.last_clock_ns is not None and current < self.last_clock_ns:
            self.reset_map(clear_poses=True)
            self.get_logger().info('Simulation clock reset; cleared accumulated map')
        self.last_clock_ns = current

    def pose_callback(self, message):
        """Buffer true world model poses, never estimated odometry."""
        if message.header.frame_id not in ('', self.world_frame):
            self.warn(
                f'Ground truth frame {message.header.frame_id!r} does not match '
                f'world_frame {self.world_frame!r}; pose ignored')
            return
        stamp = stamp_ns(message.header.stamp)
        if stamp < 0:
            return
        if self.last_clock_ns is not None:
            # Ignore previous-epoch samples arriving after a simulation reset.
            if stamp > self.last_clock_ns + max(self.max_pose_gap_ns, 1_000_000_000):
                self.warn('Ground truth is ahead of simulation clock; pose ignored')
                return
        elif (self.poses.newest_stamp_ns is not None
              and stamp < self.poses.newest_stamp_ns - self.reset_threshold_ns):
            # Also support testing without a /clock bridge.
            self.reset_map(clear_poses=True)
        position = message.pose.position
        quaternion = message.pose.orientation
        try:
            self.poses.add(
                stamp, (position.x, position.y, position.z),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        except ValueError as error:
            self.warn(f'Invalid ground truth pose: {error}')
            return
        display_pose = deepcopy(message)
        display_pose.header.frame_id = self.world_frame
        self.pose_pub.publish(display_pose)
        self.process_pending()

    def range_callback(self, position, message):
        """Queue a finite distance until a pose at its acquisition time exists."""
        if message.header.frame_id != f'sonar_{position}_link':
            self.warn(f'Unexpected frame for {position} sonar; reading ignored')
            return
        distance, lower, upper = message.range, message.min_range, message.max_range
        if (not all(math.isfinite(value) for value in (distance, lower, upper))
                or lower < 0 or upper < lower or distance <= 0
                or not lower <= distance <= upper):
            return
        stamp = stamp_ns(message.header.stamp)
        if stamp < 0:
            return
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.warn('Pose wait queue full; discarded oldest pending reading')
        self.pending.append(PendingReading(
            self.sensor_names[position], distance, lower, upper, stamp, time.monotonic()))
        self.process_pending()

    def process_pending(self):
        """Project ready readings and discard only unresolvable pending data."""
        waiting = deque()
        now = time.monotonic()
        for reading in self.pending:
            if now - reading.received_at > self.pending_timeout:
                self.warn('Timed out waiting for ground truth at sonar timestamp')
                continue
            if (self.poses.oldest_stamp_ns is not None
                    and reading.stamp < self.poses.oldest_stamp_ns):
                continue
            pose = self.poses.sample(reading.stamp, max_gap_ns=self.max_pose_gap_ns)
            if pose is None:
                waiting.append(reading)
                continue
            position, quaternion = pose
            if self.accumulator.add(
                    reading.sensor, reading.distance, reading.min_range,
                    reading.max_range, position, quaternion):
                self.dirty = True
                self.latest_point_stamp = max(self.latest_point_stamp, reading.stamp)
        self.pending = waiting

    def reset_map(self, clear_poses=False):
        """Publish an empty snapshot so RViz clears immediately as well."""
        self.accumulator.clear()
        self.pending.clear()
        if clear_poses:
            self.poses.clear()
        self.latest_point_stamp = 0
        self.dirty = True
        self.publish_cloud()

    def clear_callback(self, request, response):
        """Clear accumulated points on demand without losing current pose history."""
        self.reset_map()
        response.success = True
        response.message = 'Cleared accumulated sonar map'
        return response

    def publish_cloud(self):
        """Publish a full world-frame snapshot; no point expiration or truncation."""
        header = Header(frame_id=self.world_frame)
        header.stamp.sec, header.stamp.nanosec = divmod(
            self.latest_point_stamp, 1_000_000_000)
        points = np.asarray(self.accumulator.points, dtype=np.float32).reshape(-1, 3)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.dirty = False

    def tick(self):
        """Expire pending readings and refresh the retained full map when changed."""
        self.process_pending()
        if self.dirty:
            self.publish_cloud()


def main(args=None):
    """Run the mapper until interrupted, preserving every point for this run."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PointCloudMapper()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
