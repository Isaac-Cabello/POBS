"""Publish simulated water depth and pressure from the vehicle's world pose.

The downward sonar measures bottom clearance and has its own Range topic.
Its readings must not be used as water depth without a known seabed elevation.
"""

from copy import deepcopy
import math

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import FluidPressure


class Depth(Node):
    """Keep the original depth/pressure formulas with stamped depth output."""

    def __init__(self):
        super().__init__('pobs_depth_sensor')
        self.depth = self.create_publisher(PointStamped, '/pobs/depth', 10)
        self.pressure = self.create_publisher(FluidPressure, '/pobs/pressure', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/model/pobs/pose', self.update, 10)

    def update(self, msg):
        """Use the original positive-down depth convention and source timestamp."""
        if not math.isfinite(msg.pose.position.z):
            return
        depth_m = max(0.0, -msg.pose.position.z)
        depth = PointStamped()
        depth.header = deepcopy(msg.header)
        depth.point.z = depth_m
        self.depth.publish(depth)

        pressure = FluidPressure()
        pressure.header = deepcopy(msg.header)
        pressure.header.frame_id = 'base_link'
        pressure.fluid_pressure = 101325.0 + 1000.0 * 9.81 * depth_m
        pressure.variance = 0.0
        self.pressure.publish(pressure)


def main(args=None):
    """Run the simulated depth and pressure publisher."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Depth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
