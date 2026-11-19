
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class ESPTestNode(Node):
    def __init__(self):
        super().__init__('esp_test_node')

        self.declare_parameter('serial_enabled', False)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('deadband', 0.01)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('thruster_topics', [
            '/model/pobs/joint/thruster_0_joint/cmd_thrust',
            '/model/pobs/joint/thruster_1_joint/cmd_thrust',
            '/model/pobs/joint/thruster_2_joint/cmd_thrust',
            '/model/pobs/joint/thruster_3_joint/cmd_thrust',
        ])

        self.serial_enabled = bool(
            self.get_parameter('serial_enabled').value
        )
        self.serial_port = str(
            self.get_parameter('serial_port').value
        )
        self.baud_rate = int(
            self.get_parameter('baud_rate').value
        )
        self.deadband = float(
            self.get_parameter('deadband').value
        )
        self.timeout = float(
            self.get_parameter('command_timeout').value
        )
        topics = list(self.get_parameter('thruster_topics').value)

        if len(topics) != 4 or len(set(topics)) != 4:
            raise ValueError('Exactly four unique topics required')
        if not math.isfinite(self.deadband) or self.deadband < 0:
            raise ValueError('Invalid deadband')
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Invalid timeout')
        if self.baud_rate <= 0:
            raise ValueError('Invalid baud rate')

        self.led_states = [False] * 4
        self.last_commands = [None] * 4
        self.serial_connection = None

        if self.serial_enabled:
            import serial

            self.serial_connection = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1,
            )
            # ESP32 may reset when serial is opened.
            time.sleep(2.0)
            for i in range(4):
                self.send_led(i, False)

        self.state_sub = self.create_subscription(
            Odometry,
            '/state/odometry',
            self.state_callback,
            10,
        )

        self.thruster_subs = []
        for i, topic in enumerate(topics):
            sub = self.create_subscription(
                Float64,
                topic,
                lambda msg, index=i: self.thruster_callback(
                    msg, index
                ),
                10,
            )
            self.thruster_subs.append(sub)
            self.get_logger().info(
                f'Thruster {i} -> LED {i + 1}: {topic}'
            )

        self.timer = self.create_timer(0.1, self.periodic_update)

        self.get_logger().info(
            f'ESP test node ready, serial_enabled={self.serial_enabled}'
        )

    def state_callback(self, msg):
        self.last_odometry = msg

    def send_led(self, index, enabled):
        if self.serial_connection is None:
            return

        command = f'LED,{index},{int(enabled)}\n'
        try:
            self.serial_connection.write(command.encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self.serial_connection.close()
            self.serial_connection = None

    def update_led(self, index, enabled):
        if self.led_states[index] == enabled:
            return

        self.led_states[index] = enabled
        self.send_led(index, enabled)

        status = 'ON' if enabled else 'OFF'
        self.get_logger().info(f'Thruster {index} LED {status}')

    def thruster_callback(self, msg, index):
        now = time.monotonic()
        value = float(msg.data)

        if not math.isfinite(value):
            self.last_commands[index] = None
            self.update_led(index, False)
            return

        self.last_commands[index] = now
        self.update_led(index, abs(value) > self.deadband)

    def check_timeout(self):
        now = time.monotonic()

        for i, last_time in enumerate(self.last_commands):
            if last_time is None or now - last_time > self.timeout:
                self.update_led(i, False)
    def periodic_update(self):
        self.check_timeout()
        if self.serial_connection is not None:
            for i, state in enumerate(self.led_states):
                self.send_led(i, state)

    def destroy_node(self):
        if self.serial_connection is not None:
            for i in range(4):
                self.send_led(i, False)
            self.serial_connection.close()

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ESPTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
