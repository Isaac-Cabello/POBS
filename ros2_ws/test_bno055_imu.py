import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class MockBNO055(Node):
    def __init__(self):
        super().__init__('mock_bno055')

        self.pub = self.create_publisher(Imu, '/pobs/imu', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)

    def publish_imu(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pobs_imu'

        msg.orientation.w = 1.0

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        msg.linear_acceleration.x = 1.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MockBNO055()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
