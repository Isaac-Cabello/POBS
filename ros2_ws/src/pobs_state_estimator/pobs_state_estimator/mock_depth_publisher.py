import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class MockDepthPublisher(Node):
    def __init__(self):
        super().__init__('mock_depth_publisher')

        self.publisher = self.create_publisher(
            PointStamped,
            '/pobs/depth',
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_depth,
        )

        self.get_logger().info(
            'Mock depth publisher started at 10 Hz'
        )

    def publish_depth(self):
        msg = PointStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.point.x = 0.0
        msg.point.y = 0.0
        msg.point.z = 3.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockDepthPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()