from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryComparison(Node):
    """Compare estimated AUV motion against Gazebo ground truth."""

    def __init__(self):
        super().__init__('trajectory_comparison')
        self.get_logger().info('Trajectory comparison started')

        self.latest_estimated = None
        self.latest_truth = None
        self.match_tolerance_ns = 50_000_000  # 50 milliseconds; tune as needed.
        self.declare_parameter('reset_settle_s', 0.25)
        reset_settle_s = float(self.get_parameter('reset_settle_s').value)
        if not math.isfinite(reset_settle_s) or reset_settle_s < 0:
            raise ValueError('reset_settle_s must be finite and nonnegative')
        self.reset_settle_ns = round(reset_settle_s * 1_000_000_000)
        self.reset_ready_ns = None

        self.max_path_points = 2000
        # The current estimator uses world-aligned axes and absolute depth,
        # but starts its horizontal position at zero. Match lake.sdf's spawn
        # x/y; z stays zero because the estimator already reports -depth.
        self.declare_parameter('publish_odom_transform', True)
        self.declare_parameter('odom_offset_x', -12.0)
        self.declare_parameter('odom_offset_y', -8.0)
        self.odom_offset_x = float(self.get_parameter('odom_offset_x').value)
        self.odom_offset_y = float(self.get_parameter('odom_offset_y').value)
        if self.get_parameter('publish_odom_transform').value:
            self.odom_transform_broadcaster = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.frame_id = 'lake'
            transform.child_frame_id = 'odom'
            transform.transform.translation.x = self.odom_offset_x
            transform.transform.translation.y = self.odom_offset_y
            transform.transform.rotation.w = 1.0
            self.odom_transform_broadcaster.sendTransform(transform)
        self.last_estimated_stamp_ns = None
        self.last_truth_stamp_ns = None
        self.last_clock_ns = None
        self.estimated_path = Path()
        self.ground_truth_path = Path()
        self.estimated_path.header.frame_id = 'lake'
        self.ground_truth_path.header.frame_id = 'lake'
        path_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/state/odometry',
            self.estimated_odometry_callback,
            10
        )

        self.true_odom_subscription = self.create_subscription(
            PoseStamped,
            '/model/pobs/pose',
            self.ground_truth_odometry_callback,
            10
        )

        self.estimated_path_publisher = self.create_publisher(
            Path,
            '/visualization/estimated_path',
            path_qos
        )

        self.ground_truth_path_publisher = self.create_publisher(
            Path,
            '/visualization/ground_truth_path',
            path_qos
        )

        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.lake_marker_publisher = self.create_publisher(
            MarkerArray, '/visualization/lake', path_qos)
        self.lake_markers = self.make_lake_markers()
        self.publish_lake()
        # Refresh after RViz clears its scene on a simulation-time reset.
        self.lake_timer = self.create_timer(10.0, self.publish_lake)

    @staticmethod
    def make_lake_markers() -> MarkerArray:
        """Reuse the Gazebo terrain meshes in the lake coordinate frame."""
        markers = MarkerArray()
        for marker_id, (name, color) in enumerate([
                ('terrain', (0.32, 0.38, 0.20, 1.0)),
                ('water', (0.02, 0.28, 0.70, 0.12))]):
            marker = Marker()
            marker.header.frame_id = 'lake'
            # Stamp zero keeps the static geometry independent of sim time.
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            # lake.sdf places lake_terrain at the origin with no rotation;
            # its mesh geometry uses unit scale in model.sdf.
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.mesh_resource = (
                'package://pobs_gazebo/models/lake_terrain/meshes/'
                f'{name}.obj')
            marker.mesh_use_embedded_materials = False
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            markers.markers.append(marker)
        return markers

    def publish_lake(self) -> None:
        """Publish static terrain even when no estimated odometry exists."""
        self.lake_marker_publisher.publish(self.lake_markers)

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        """Convert a ROS timestamp to integer nanoseconds."""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def clock_callback(self, msg: Clock) -> None:
        """Clear the comparison immediately when simulation time rewinds."""
        stamp_ns = self.stamp_to_ns(msg.clock)
        if self.last_clock_ns is not None and stamp_ns < self.last_clock_ns:
            self.reset_comparison()
            # The unchanged estimator can initially stamp its cached old depth
            # with the new simulation time. Allow the sensor pipeline to settle.
            self.reset_ready_ns = stamp_ns + self.reset_settle_ns
        self.last_clock_ns = stamp_ns

    def accept_after_reset(self, stamp_ns: int) -> bool:
        """Skip reset transients and queued messages from the previous run."""
        if self.reset_ready_ns is None:
            return True
        if self.last_clock_ns is None or self.last_clock_ns < self.reset_ready_ns:
            return False
        return (self.reset_ready_ns <= stamp_ns
                <= self.last_clock_ns + self.match_tolerance_ns)

    def estimated_odometry_callback(self, msg: Odometry) -> None:
        stamp_ns = self.stamp_to_ns(msg.header.stamp)
        if not self.accept_after_reset(stamp_ns):
            return
        if (self.last_estimated_stamp_ns is not None
                and stamp_ns < self.last_estimated_stamp_ns):
            self.reset_comparison()
        self.last_estimated_stamp_ns = stamp_ns
        self.latest_estimated = msg
        self.process_matched_samples()


    def ground_truth_odometry_callback(self, msg: PoseStamped) -> None:
        stamp_ns = self.stamp_to_ns(msg.header.stamp)
        if not self.accept_after_reset(stamp_ns):
            return
        if (self.last_truth_stamp_ns is not None
                and stamp_ns < self.last_truth_stamp_ns):
            self.reset_comparison()
        self.last_truth_stamp_ns = stamp_ns
        self.latest_truth = msg
        self.process_matched_samples()

    def process_matched_samples(self) -> None:
        """Compare the latest messages when their timestamps are close enough."""
        if self.latest_estimated is None or self.latest_truth is None:
            return

        estimated_ns = self.stamp_to_ns(self.latest_estimated.header.stamp)
        truth_ns = self.stamp_to_ns(self.latest_truth.header.stamp)

        if abs(estimated_ns - truth_ns) > self.match_tolerance_ns:
            return

        estimated = self.latest_estimated
        truth = self.latest_truth
        # Consume the pair so a later callback cannot reuse either sample.
        self.latest_estimated = None
        self.latest_truth = None
        self.compare_pair(estimated, truth)

    def compare_pair(self, estimated: Odometry, ground_truth: PoseStamped) -> None:
        """Convert the estimate to lake before publishing either trail."""
        if ground_truth.header.frame_id != 'lake':
            self.get_logger().warn(
                'Ground-truth pose must use the lake frame',
                throttle_duration_sec=2.0)
            return
        estimated_pose = PoseStamped(
            header=deepcopy(estimated.header),
            pose=deepcopy(estimated.pose.pose))
        if estimated.header.frame_id == 'odom':
            # Apply the same configured translation as lake -> odom here.
            # RViz then needs no odom lookup while its TF buffer resets.
            estimated_pose.pose.position.x += self.odom_offset_x
            estimated_pose.pose.position.y += self.odom_offset_y
            estimated_pose.header.frame_id = 'lake'
        elif estimated.header.frame_id != 'lake':
            self.get_logger().warn(
                f'Unsupported estimated frame: {estimated.header.frame_id!r}',
                throttle_duration_sec=2.0)
            return

        self.estimated_path.header = deepcopy(estimated_pose.header)
        self.ground_truth_path.header = deepcopy(ground_truth.header)

        if len(self.estimated_path.poses) >= self.max_path_points:
            self.estimated_path.poses.pop(0)

        if len(self.ground_truth_path.poses) >= self.max_path_points:
            self.ground_truth_path.poses.pop(0)

        self.estimated_path.poses.append(estimated_pose)
        self.ground_truth_path.poses.append(deepcopy(ground_truth))

        self.publish_paths()

    def publish_paths(self) -> None:
        self.get_logger().debug(
            f'Publishing {len(self.estimated_path.poses)} estimated points '
            f'and {len(self.ground_truth_path.poses)} ground-truth points')
        self.estimated_path_publisher.publish(self.estimated_path)
        self.ground_truth_path_publisher.publish(self.ground_truth_path)

    def publish_position_error(self) -> None:
        pass

    def reset_comparison(self) -> None:
        """Clear pending samples and publish empty trails to clear RViz."""
        self.latest_estimated = None
        self.latest_truth = None
        self.last_estimated_stamp_ns = None
        self.last_truth_stamp_ns = None
        self.last_clock_ns = None
        self.reset_ready_ns = None
        self.estimated_path.poses.clear()
        self.ground_truth_path.poses.clear()
        stamp = self.get_clock().now().to_msg()
        self.estimated_path.header.stamp = stamp
        self.ground_truth_path.header.stamp = stamp
        self.publish_paths()
        self.get_logger().info('Comparison reset; both paths cleared')
        self.publish_lake()


def main(args=None):
    """Run the trajectory comparison node."""
    rclpy.init(args=args)
    node = TrajectoryComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
