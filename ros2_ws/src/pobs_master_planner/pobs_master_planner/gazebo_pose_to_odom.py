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
        self.prev_pos = None
        self.prev_time = None
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
        #unknown v must not be interpretated as 0
        msg.twist.twist.linear.x = float('nan')
        # Deliberately do not infer velocity from pose or provide fake
        # high-confidence covariance. This adapter is for planner logic.
        msg.pose.covariance[0] = 0.0
        msg.pose.covariance[7] = 0.0
        #estimate body-frame fwd v from gazebo pose
        now = self.get_clock().now().nanoseconds / 1e9
        if self.prev_pos is not None:
            dt = now - self.prev_time
            if 0.01 <= dt <= 0.5:
                vx_world = (x-self.prev_pos[0])/dt
                vy_world = (y-self.prev_pos[1])/dt
                siny = 2.0 * (q.w*q.z + q.x*q.y)
                cosy = 1.0 - 2.0 * (q.y*q.y +q.z*q.z)
                yaw = math.atan2(siny,cosy)

                vx_body = (math.cos(yaw)* vx_world+ math.sin(yaw)*vy_world)
                msg.twist.twist.linear.x = vx_body
        self.prev_pos = (x,y)
        self.prev_time = now
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
