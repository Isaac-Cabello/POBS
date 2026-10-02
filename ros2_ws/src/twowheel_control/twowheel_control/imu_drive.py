import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class ImuDrive(Node):
    def __init__(self):
        super().__init__('imu_drive')

        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('max_yaw_rate', 0.50)
        self.declare_parameter('max_forward_accel', 1.00)

        self.cruise_speed = float(
            self.get_parameter('cruise_speed').value
        )
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value
        )
        self.max_forward_accel = float(
            self.get_parameter('max_forward_accel').value
        )

        self.forward_accel = 0.0
        self.yaw_rate = 0.0

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/pobs/imu_filtered',
            self.imu_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Twist,
            '/model/2-wheeled_bot/cmd_vel',
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Publishing control commands to /model/2-wheeled_bot/cmd_vel'
        )

    def imu_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn('Ignoring incomplete IMU-filter message')
            return

        self.forward_accel = msg.data[0]
        self.yaw_rate = msg.data[2]

    def control_loop(self) -> None:
        command = Twist()

        if abs(self.yaw_rate) > self.max_yaw_rate:
            command.linear.x = 0.0
            command.angular.z = 0.0
            self.get_logger().warn('Stopping: yaw rate exceeds safe threshold')
        elif abs(self.forward_accel) > self.max_forward_accel:
            command.linear.x = 0.0
            command.angular.z = 0.0
            self.get_logger().warn(
                'Stopping: forward acceleration exceeds safe threshold'
            )
        else:
            command.linear.x = self.cruise_speed
            command.angular.z = 0.0

        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()