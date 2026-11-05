import math
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ESPTCPClient(Node):
    def __init__(self):
        super().__init__('esp_tcp_client')
        self.declare_parameter('pi_ip', '127.0.0.1')
        self.declare_parameter('pi_port', 8765)
        self.declare_parameter('deadband', 0.01)
        self.declare_parameter('command_timeout', 0.5)
        self.pi_ip = self.get_parameter('pi_ip').value
        self.pi_port = int(self.get_parameter('pi_port').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.led_states = [False] * 4
        self.last_commands = [None] * 4
        self.sock = None
        topics = [
            '/model/pobs/joint/thruster_0_joint/cmd_thrust',
            '/model/pobs/joint/thruster_1_joint/cmd_thrust',
            '/model/pobs/joint/thruster_2_joint/cmd_thrust',
            '/model/pobs/joint/thruster_3_joint/cmd_thrust',
        ]

        self.subscriptions_list = []
        for i, topic in enumerate(topics):
            sub = self.create_subscription(
                Float64,
                topic,
                lambda msg, index=i: self.thruster_callback(
                    msg, index
                ),
                10,
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(
                f'Thruster {i} -> LED {i + 1}'
            )
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('ESP TCP client started')
    def connect(self):
        try:
            self.sock = socket.create_connection(
                (self.pi_ip, self.pi_port),
                timeout=1,
            )
            self.sock.settimeout(1)
            self.get_logger().info(
                f'Connected to Raspberry Pi: {self.pi_ip}'
            )
        except OSError:
            self.sock = None
    def send_led(self, index, state):
        if self.sock is None:
            return
        try:
            command = f'LED,{index},{int(state)}\n'
            self.sock.sendall(command.encode('ascii'))
        except OSError:
            self.sock.close()
            self.sock = None
    def thruster_callback(self, msg, index):
        value = float(msg.data)
        if not math.isfinite(value):
            self.last_commands[index] = None
            self.led_states[index] = False
            return
        self.last_commands[index] = time.monotonic()
        self.led_states[index] = abs(value) > self.deadband
    def update(self):
        now = time.monotonic()
        for i in range(4):
            last = self.last_commands[i]
            if last is None or now - last > self.timeout:
                self.led_states[i] = False
        if self.sock is None:
            self.connect()
        if self.sock is not None:
            for i, state in enumerate(self.led_states):
                self.send_led(i, state)
    def destroy_node(self):
        if self.sock is not None:
            for i in range(4):
                self.send_led(i, False)
            self.sock.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESPTCPClient()
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
