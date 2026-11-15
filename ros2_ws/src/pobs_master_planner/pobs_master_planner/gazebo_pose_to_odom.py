"""Simulator-only adapter: Gazebo ground-truth Pose -> relative XY Odometry.

This is for Master Planner dry-run testing. Do not treat the output as a
measured position estimate for the physical robot.
"""
import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GazeboPoseToOdom(Node):
    def __init__(self):
        super().__init__('gazebo_pose_to_odom')
        self.declare_parameter('pose_topic', '/model/pobs/pose')
        self.declare_parameter('odom_topic', '/planner/sim_odometry')
        self.origin_xy = None
        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.subscription = self.create_subscription(
            Pose, pose_topic, self.on_pose, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Simulation ONLY: {pose_topic} -> {odom_topic}; '
            'XY origin set from first Gazebo pose'
        )

    def on_pose(self, pose):
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation
        if not all(math.isfinite(v) for v in (
            x, y, z, q.x, q.y, q.z, q.w
        )):
            self.get_logger().warning('Ignoring non-finite Gazebo pose')
            return

        if self.origin_xy is None:
            self.origin_xy = (x, y)
            self.get_logger().info(
                f'Simulation XY origin established at Gazebo '
                f'({x:.3f}, {y:.3f})'
            )

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x - self.origin_xy[0]
        msg.pose.pose.position.y = y - self.origin_xy[1]
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = q
        # Deliberately do not infer velocity from pose or provide fake
        # high-confidence covariance. This adapter is for planner logic.
        msg.pose.covariance[0] = 0.0
        msg.pose.covariance[7] = 0.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
