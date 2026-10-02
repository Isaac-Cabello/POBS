import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


class ImuFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')

        self.declare_parameter('accel_deadband', 0.10)
        self.declare_parameter('yaw_rate_deadband', 0.05)

        self.accel_deadband = float(
            self.get_parameter('accel_deadband').value
        )
        self.yaw_rate_deadband = float(
            self.get_parameter('yaw_rate_deadband').value
        )

        self.subscription = self.create_subscription(
            Imu,
            '/model/2-wheeled_bot/imu',
            self.imu_callback,
            50,
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/pobs/imu_filtered',
            10,
        )

        self.get_logger().info(
            'Listening to /model/2-wheeled_bot/imu '
            'and publishing /pobs/imu_filtered'
        )

    @staticmethod
    def deadband(value: float, tolerance: float) -> float:
        return 0.0 if abs(value) < tolerance else value

    def imu_callback(self, msg: Imu) -> None:
        forward_accel = self.deadband(
            msg.linear_acceleration.x,
            self.accel_deadband,
        )

        lateral_accel = self.deadband(
            msg.linear_acceleration.y,
            self.accel_deadband,
        )

        yaw_rate = self.deadband(
            msg.angular_velocity.z,
            self.yaw_rate_deadband,
        )

        output = Float64MultiArray()
        output.data = [
            forward_accel,
            lateral_accel,
            yaw_rate,
        ]

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()