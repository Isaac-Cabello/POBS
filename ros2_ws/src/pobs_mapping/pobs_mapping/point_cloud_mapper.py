"""Map timestamped sonar measurements in a fixed world coordinate frame."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import os
import time

from geometry_msgs.msg import Point, PoseStamped
import numpy as np
from pobs_mapping.core import PointAccumulator, PoseBuffer
from pobs_mapping.elevation_color import elevation_rgb
from pobs_mapping.footprint import FootprintMap
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
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


POSITIONS = ('left', 'right', 'middle', 'depth')


def stamp_ns(stamp):
    """Convert an acquisition timestamp into integer nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass
class PendingReading:
    """A valid reading waiting for bracketing ground-truth poses."""

    sensor: str
    stamp: int
    received_at: float
    measurement: tuple[float, float, float] | np.ndarray


class PointCloudMapper(Node):
    """Project sonar readings using SDF mounts and timestamped ground truth."""

    def __init__(self, **kwargs):
        super().__init__('point_cloud_mapper', **kwargs)
        workspace = os.environ.get('POBS_WORKSPACE', os.path.expanduser('~/ros2_ws'))
        defaults = {
            'model_path': os.path.join(
                workspace, 'src', 'pobs_gazebo', 'models', 'pobs', 'model.sdf'),
            'world_frame': 'lake',
            'pose_topic': '/model/pobs/pose',
            'mapping_method': 'points',
            'footprint_cell_size_m': 1.0,
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
        self.method = self.get_parameter('mapping_method').value
        if self.method not in ('points', 'footprints'):
            raise ValueError('mapping_method must be points or footprints')
        suffix = '_beam' if self.method == 'footprints' else ''
        self.sensor_names = {position: f'sonar_{position}{suffix}' for position in POSITIONS}
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        transforms = load_sensor_transforms(model_path, self.sensor_names.values())
        if self.method == 'footprints':
            self.accumulator = FootprintMap(
                transforms, self.positive_parameter('footprint_cell_size_m'))
        else:
            self.accumulator = PointAccumulator(transforms)
        self.poses = PoseBuffer(max_age_ns=int(history * 1e9))
        self.pending = deque()
        self.last_clock_ns = None
        self.latest_map_stamp = 0
        self.dirty = True

        # A late RViz subscriber receives the latest complete accumulated map.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.method == 'footprints':
            self.map_pub = self.create_publisher(Marker, '/pobs/mapping/footprints', map_qos)
            message_type, topic_suffix = PointCloud2, 'beam/points'
            callback = self.beam_callback
        else:
            self.map_pub = self.create_publisher(PointCloud2, '/pobs/mapping/points', map_qos)
            message_type, topic_suffix = Range, 'range'
            callback = self.range_callback
        self.pose_pub = self.create_publisher(
            PoseStamped, '/pobs/mapping/ground_truth_pose', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self.pose_callback, qos_profile_sensor_data)
        self.range_subscriptions = [
            self.create_subscription(
                message_type, f'/pobs/sonar/sonar_{position}/{topic_suffix}',
                lambda message, position=position: callback(position, message),
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
            f'mapping sonar {self.method} in {self.world_frame}')

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
        """Clear the map and pose history when simulation time goes backward."""
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
        self.queue_reading(position, stamp_ns(message.header.stamp), (distance, lower, upper))

    def beam_callback(self, position, message):
        """Read sensor-frame XYZ samples from one Gazebo beam sweep."""
        try:
            hits = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=False)
        except (AssertionError, KeyError, ValueError) as error:
            self.warn(f'Invalid {position} sonar beam: {error}')
            return
        self.queue_reading(position, stamp_ns(message.header.stamp), hits.reshape(-1, 3))

    def queue_reading(self, position, stamp, measurement):
        """Wait for the ground-truth pose at the reading's timestamp."""
        if stamp < 0 or (
                self.last_clock_ns is not None
                and stamp > self.last_clock_ns + max(self.max_pose_gap_ns, 1_000_000_000)):
            return
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.warn('Pose wait queue full; discarded oldest pending reading')
        self.pending.append(PendingReading(
            self.sensor_names[position], stamp, time.monotonic(), measurement))
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
            if self.method == 'footprints':
                changed = self.accumulator.add(
                    reading.sensor, reading.measurement, position, quaternion, reading.stamp)
            else:
                changed = self.accumulator.add(
                    reading.sensor, *reading.measurement, position, quaternion)
            if changed:
                self.dirty = True
                self.latest_map_stamp = max(self.latest_map_stamp, reading.stamp)
        self.pending = waiting

    def reset_map(self, clear_poses=False):
        """Publish an empty snapshot so RViz clears immediately as well."""
        self.accumulator.clear()
        self.pending.clear()
        if clear_poses:
            self.poses.clear()
        self.latest_map_stamp = 0
        self.dirty = True
        self.publish_map()

    def clear_callback(self, request, response):
        """Clear accumulated points on demand without losing current pose history."""
        self.reset_map()
        response.success = True
        response.message = 'Cleared accumulated sonar map'
        return response

    def publish_map(self):
        """Publish the complete world-frame map for current and late subscribers."""
        header = Header(frame_id=self.world_frame)
        header.stamp.sec, header.stamp.nanosec = divmod(
            self.latest_map_stamp, 1_000_000_000)
        if self.method == 'footprints':
            self.publish_footprints(header)
        else:
            points = np.asarray(self.accumulator.points, dtype=np.float32).reshape(-1, 3)
            self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.dirty = False

    def publish_footprints(self, header):
        """Render the complete accumulated occupied voxel map."""
        marker = Marker(header=header, ns='sonar_footprints', type=Marker.CUBE_LIST)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = self.accumulator.cell_size
        marker.scale.z = self.accumulator.cell_size
        marker.color.a = 1.0
        marker.action = Marker.ADD if self.accumulator.cells else Marker.DELETE
        points, colors = [], []
        for x, y, z in self.accumulator.elevations():
            points.append(Point(x=x, y=y, z=z))
            r, g, b = elevation_rgb(z)
            colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))
        marker.points = points
        marker.colors = colors
        self.map_pub.publish(marker)

    def tick(self):
        """Expire pending readings and refresh the retained full map when changed."""
        self.process_pending()
        if self.dirty:
            self.publish_map()


def main(args=None):
    """Run the mapper until interrupted."""
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
