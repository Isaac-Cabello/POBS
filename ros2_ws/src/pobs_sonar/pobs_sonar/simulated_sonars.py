"""Adapt the four mounted Gazebo ray sensors to individual Range topics."""

import math

from pobs_sonar.mounting import SONAR_MOUNTS
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, Range


def range_from_scan(scan, frame_id, min_range=0.05, max_range=6.0,
                    field_of_view=math.radians(16.0), preserve_no_return=False):
    """Keep valid ray distances and the simulator's acquisition timestamp."""
    distances = [float(value) for value in scan.ranges
                 if math.isfinite(value) and min_range <= value <= max_range]
    if not distances:
        # Preserve the original downward ray's explicit no-hit/too-close values.
        if (not preserve_no_return or len(scan.ranges) != 1
                or not math.isinf(scan.ranges[0])):
            return None
        distances = [scan.ranges[0]]
    message = Range()
    message.header.stamp = scan.header.stamp
    message.header.frame_id = frame_id
    message.radiation_type = Range.ULTRASOUND
    message.field_of_view = field_of_view
    message.min_range = min_range
    message.max_range = max_range
    message.range = min(distances)
    return message


class SimulatedSonars(Node):
    """Publish four distance topics, keeping the original downward simulation."""

    def __init__(self):
        super().__init__('simulated_sonars')
        self.publishers_by_position = {}
        self.subscriptions_by_position = {}
        for position in SONAR_MOUNTS:
            prefix = f'/pobs/sonar/sonar_{position}'
            self.publishers_by_position[position] = self.create_publisher(
                Range, prefix + '/range', qos_profile_sensor_data)
            self.subscriptions_by_position[position] = self.create_subscription(
                LaserScan, prefix + '/scan',
                lambda scan, position=position: self.update(position, scan),
                qos_profile_sensor_data)

    def update(self, position, scan):
        """Forward one valid distance for the sensor that produced the scan."""
        if position == 'depth':
            message = range_from_scan(
                scan, 'sonar_depth_link', min_range=0.15, max_range=30.0,
                field_of_view=0.0, preserve_no_return=True)
        else:
            message = range_from_scan(scan, f'sonar_{position}_link')
        if message is not None:
            self.publishers_by_position[position].publish(message)


def main(args=None):
    """Run the four-sensor simulation adapter."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimulatedSonars()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
