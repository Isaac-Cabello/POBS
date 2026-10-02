import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.declare_parameter('accel_deadband_m_s2', 0.05)
        self.declare_parameter('max_dt_s', 0.20)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.accel_deadband = float(
            self.get_parameter('accel_deadband_m_s2').value
        )
        self.max_dt = float(self.get_parameter('max_dt_s').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0

        self.vx_m_s = 0.0
        self.vy_m_s = 0.0

        self.last_time = None
        self.latest_accel_x = 0.0
        self.latest_accel_y = 0.0
        self.latest_yaw_rate = 0.0
        self.received_measurement = False

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/twowheel/imu_filtered',
            self.imu_callback,
            50,
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/twowheel/odom_estimate',
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_odometry,
        )

        self.get_logger().info(
            'Dead reckoning started: /twowheel/imu_filtered '
            '-> /twowheel/odom_estimate'
        )

    def imu_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn(
                'Expected [accel_x, accel_y, yaw_rate] from imu_filter'
            )
            return

        now = time.monotonic()

        accel_x_body = float(msg.data[0])
        accel_y_body = float(msg.data[1])
        yaw_rate = float(msg.data[2])

        if abs(accel_x_body) < self.accel_deadband:
            accel_x_body = 0.0

        if abs(accel_y_body) < self.accel_deadband:
            accel_y_body = 0.0

        self.latest_accel_x = accel_x_body
        self.latest_accel_y = accel_y_body
        self.latest_yaw_rate = yaw_rate
        self.received_measurement = True

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        self.yaw_rad += yaw_rate * dt
        self.yaw_rad = math.atan2(
            math.sin(self.yaw_rad),
            math.cos(self.yaw_rad),
        )

        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)

        accel_x_world = (
            accel_x_body * cos_yaw - accel_y_body * sin_yaw
        )
        accel_y_world = (
            accel_x_body * sin_yaw + accel_y_body * cos_yaw
        )

        self.x_m += self.vx_m_s * dt + 0.5 * accel_x_world * dt * dt
        self.y_m += self.vy_m_s * dt + 0.5 * accel_y_world * dt * dt

        self.vx_m_s += accel_x_world * dt
        self.vy_m_s += accel_y_world * dt

    def publish_odometry(self) -> None:
        if not self.received_measurement:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x_m
        odom.pose.pose.position.y = self.y_m
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.yaw_rad / 2.0),
            w=math.cos(self.yaw_rad / 2.0),
        )

        odom.twist.twist.linear.x = self.vx_m_s
        odom.twist.twist.linear.y = self.vy_m_s
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.latest_yaw_rate

        self.odom_publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()