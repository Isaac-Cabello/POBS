"""Adapt simulated pose to the estimator's stamped positive-depth interface."""

from copy import deepcopy
import math

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure


class Depth(Node):
    """Publish depth and pressure at the source pose's simulation timestamp."""

    def __init__(self):
        super().__init__('pobs_depth_sensor')
        self.depth = self.create_publisher(PointStamped, '/pobs/depth', 10)
        self.pressure = self.create_publisher(FluidPressure, '/pobs/pressure', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/model/pobs/pose', self.update, 10)

    def update(self, msg: PoseStamped) -> None:
        """Store positive depth in point.z, as required by StateEstimator."""
        if not math.isfinite(msg.pose.position.z):
            return
        depth_m = max(0.0, -msg.pose.position.z)
        depth = PointStamped()
        depth.header = deepcopy(msg.header)
        # This topic carries a depth measurement, not a robot position.
        depth.point.z = depth_m
        self.depth.publish(depth)

        pressure = FluidPressure()
        pressure.header = deepcopy(msg.header)
        pressure.header.frame_id = 'base_link'
        pressure.fluid_pressure = 101325.0 + 1000.0 * 9.81 * depth_m
        pressure.variance = 0.0
        self.pressure.publish(pressure)


def main(args=None):
    """Run the simulation depth adapter."""
    rclpy.init(args=args)
    node = Depth()
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
