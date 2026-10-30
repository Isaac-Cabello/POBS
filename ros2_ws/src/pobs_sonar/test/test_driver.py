"""Exercise actual ROS nodes and topics with emulated UART connections."""

from types import SimpleNamespace

from pobs_sonar import sen0599_driver as driver
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
import serial


def frame(distance):
    high, low = divmod(distance, 256)
    return bytes([0xff, high, low, (0xff + high + low) & 0xff])


class FakeSerial:

    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        self.closed = True


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(index):
    position = ('left', 'right', 'middle', 'depth')[index - 1]
    node = driver.Sen0599Driver(
        cli_args=['--ros-args', '-r', f'__node:=sonar_{position}',
                  '-r', '__ns:=/pobs/sonar'],
        parameter_overrides=[
            Parameter('port', value=f'/dev/fake{index}'),
            Parameter('frame_id', value=f'sonar_{position}_link'),
            Parameter('topic', value=f'sonar_{position}/range'),
            Parameter('phase_offset_s', value=(index - 1) * 0.1),
            Parameter('min_range_m', value=0.15 if position == 'depth' else 0.05),
        ])
    node.timer.cancel()
    return node


def test_four_nodes_publish_independent_meter_ranges(ros, monkeypatch):
    now = [0.005]
    monkeypatch.setattr(driver, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    ports = {}

    def open_port(port, baudrate, **kwargs):
        assert baudrate == 115200
        assert kwargs['timeout'] == 0
        ports[port] = FakeSerial()
        return ports[port]

    monkeypatch.setattr(driver.serial, 'Serial', open_port)
    nodes = [make_node(index) for index in (1, 2, 3, 4)]
    listener = Node('test_sonar_listener')
    received = {index: [] for index in (1, 2, 3, 4)}
    subscriptions = [listener.create_subscription(
        Range, f'/pobs/sonar/sonar_{position}/range',
        lambda msg, index=index: received[index].append(msg),
        qos_profile_sensor_data)
        for index, position in enumerate(('left', 'right', 'middle', 'depth'), 1)]
    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    try:
        for index, node in enumerate(nodes, 1):
            now[0] = 0.005 + (index - 1) * 0.1
            for candidate in nodes:
                candidate.tick()
            port = ports[f'/dev/fake{index}']
            assert port.writes == [b'\x55']
            now[0] += 0.014
            port.rx.extend(frame(index * 1000)[:2])
            node.tick()
            port.rx.extend(frame(index * 1000)[2:])
            node.tick()
        for _ in range(50):
            executor.spin_once(timeout_sec=0.02)
            if all(received.values()):
                break
        assert len(subscriptions) == 4
        for index, messages in received.items():
            assert len(messages) == 1
            assert messages[0].range == pytest.approx(float(index))
            position = ('left', 'right', 'middle', 'depth')[index - 1]
            assert messages[0].header.frame_id == f'sonar_{position}_link'
            assert messages[0].radiation_type == Range.ULTRASOUND
            assert messages[0].min_range == pytest.approx(
                0.15 if position == 'depth' else 0.05)
            assert messages[0].max_range == pytest.approx(6.0)
    finally:
        executor.shutdown()
        listener.destroy_node()
        for node in nodes:
            node.destroy_node()
        assert all(port.closed for port in ports.values())


def test_bad_readings_timeout_and_reconnect(ros, monkeypatch):
    now = [0.005]
    monkeypatch.setattr(driver, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    port = FakeSerial()
    attempts = []

    def open_port(*args, **kwargs):
        attempts.append(now[0])
        if len(attempts) == 1:
            raise serial.SerialException('Device absent')
        return port

    monkeypatch.setattr(driver.serial, 'Serial', open_port)
    node = make_node(1)
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    try:
        node.tick()
        assert node.connection is None
        now[0] = 1.0
        node.tick()
        assert len(attempts) == 1
        now[0] = 2.155
        node.tick()
        assert len(attempts) == 2
        for distance in (0, 49, 6001, 65535):
            node.publish_distance(distance)
        assert not published
        now[0] = 2.405
        node.tick()
        assert port.writes == [b'\x55']
        port.rx.extend(b'\xff\x01\x02\x00')  # Bad checksum.
        now[0] = 2.42
        node.tick()
        assert not published
        now[0] = 2.46
        node.tick()
        assert node.deadline is None
        assert not published
        # A stale reply after a timeout must not be published as a new reading.
        port.rx.extend(frame(1234))
        now[0] = 2.47
        node.tick()
        assert not published
        node.destroy_node()
        assert port.closed
    finally:
        if not port.closed:
            node.destroy_node()
