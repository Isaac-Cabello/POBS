"""Publish one SEN0599 UART distance reading per Range message."""

import math
import time

from pobs_sonar.protocol import DistanceParser, PingSchedule
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range
import serial


class Sen0599Driver(Node):
    """Poll one sensor without blocking the executor while awaiting echoes."""

    def __init__(self, **kwargs):
        super().__init__('sen0599_driver', **kwargs)
        defaults = {
            'port': '/dev/ttyUSB0',
            'frame_id': 'sonar_link',
            'topic': 'range',
            'poll_period_s': 0.4,
            'phase_offset_s': 0.0,
            'response_timeout_s': 0.05,
            'reconnect_interval_s': 2.0,
            'field_of_view_rad': math.radians(16.0),
            'min_range_m': 0.05,
            'max_range_m': 6.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        period = float(self.get_parameter('poll_period_s').value)
        offset = float(self.get_parameter('phase_offset_s').value)
        self.schedule = PingSchedule(period, offset)
        self.response_timeout = float(
            self.get_parameter('response_timeout_s').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_s').value)
        self.field_of_view = float(
            self.get_parameter('field_of_view_rad').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        if (not all(math.isfinite(value) for value in (self.min_range, self.max_range))
                or not 0.05 <= self.min_range < self.max_range <= 6.0):
            raise ValueError('SEN0599 hardware range must stay within 0.05-6 m')
        if (not math.isfinite(self.response_timeout)
                or not 0 < self.response_timeout < period - 0.02):
            raise ValueError('Response timeout must be positive and fit the cycle')
        if (not math.isfinite(self.reconnect_interval)
                or self.reconnect_interval <= 0):
            raise ValueError('Reconnect interval must be finite and positive')
        if (not math.isfinite(self.field_of_view)
                or not 0 < self.field_of_view <= math.pi):
            raise ValueError('Field of view must be in (0, pi] radians')
        if not self.port or not self.frame_id:
            raise ValueError('Port and frame_id must be nonempty')
        self.publisher = self.create_publisher(
            Range, self.get_parameter('topic').value, qos_profile_sensor_data)
        self.parser = DistanceParser()
        self.connection = None
        self.deadline = None
        self.next_connect = 0.0
        # Acoustic scheduling uses wall monotonic time, independent of /clock.
        self.timer = self.create_timer(0.005, self.tick)
        self.get_logger().info(
            f'SEN0599 on {self.port}; distance in meters, frame {self.frame_id}')

    def disconnect(self):
        """Close a failed or stopped connection and clear pending data."""
        if self.connection is not None:
            try:
                self.connection.close()
            except (serial.SerialException, OSError):
                pass
        self.connection = None
        self.deadline = None
        self.parser.reset()

    def tick(self):
        """Reconnect, receive pending frames, and trigger in this sensor's slot."""
        now = time.monotonic()
        try:
            if self.connection is None:
                if now < self.next_connect:
                    return
                self.next_connect = now + self.reconnect_interval
                self.connection = serial.Serial(
                    self.port, 115200, timeout=0, write_timeout=0.005,
                    exclusive=True)
                self.get_logger().info(f'Connected to {self.port}')

            if self.deadline is not None:
                if now >= self.deadline:
                    self.deadline = None
                    self.parser.reset()
                    self.get_logger().warning(
                        'No valid response before timeout',
                        throttle_duration_sec=5.0)
                else:
                    count = min(self.connection.in_waiting, 256)
                    for distance_mm in self.parser.feed(
                            self.connection.read(count) if count else b''):
                        self.deadline = None
                        self.publish_distance(distance_mm)
                        # Exactly one measurement is expected per command.
                        self.parser.reset()
                        break

            if self.deadline is None and self.schedule.due(now):
                self.connection.reset_input_buffer()
                self.parser.reset()
                if self.connection.write(b'\x55') != 1:
                    raise serial.SerialException('Incomplete trigger write')
                self.deadline = time.monotonic() + self.response_timeout
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(
                f'UART error on {self.port}: {error}',
                throttle_duration_sec=5.0)
            self.disconnect()
            self.next_connect = now + self.reconnect_interval

    def publish_distance(self, distance_mm):
        """Publish valid measured ranges; zero is not a bottom observation."""
        distance_m = distance_mm / 1000.0
        if not self.min_range <= distance_m <= self.max_range:
            self.get_logger().warning(
                f'Discarding invalid distance: {distance_mm} mm',
                throttle_duration_sec=5.0)
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance_m
        self.publisher.publish(msg)

    def destroy_node(self):
        """Release the UART on shutdown."""
        self.disconnect()
        return super().destroy_node()


def main(args=None):
    """Run a single configurable sensor instance."""
    # Let Python interrupt spin before shutting down the ROS context. Automatic
    # signal shutdown can invalidate a wait set while it is being constructed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Sen0599Driver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
