"""Check stamped wrench delivery, motor output and the upstream watchdog."""
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64

from pobs_stabilizer.thruster_allocator import ThrusterAllocator


def test_stamped_wrench_to_motors_and_timeout():
    model = Path(__file__).resolve().parents[2] / 'pobs_gazebo/models/pobs/model.sdf'
    rclpy.init(args=['--ros-args', '-p', f'sdf_path:={model}', '-p', 'force_cap:=10.0'])
    allocator = ThrusterAllocator()
    observer = Node('allocator_test_observer')
    values = {}
    subscriptions = []
    for i, thruster in enumerate(allocator.thrusters):
        subscriptions.append(observer.create_subscription(
            Float64, thruster['topic'], lambda msg, index=i: values.update({index: msg.data}), 10))
    publisher = observer.create_publisher(WrenchStamped, '/cmd_wrench', 10)
    command = WrenchStamped()
    command.header.frame_id = 'base_link'
    command.wrench.force.z = 4.0
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(allocator, timeout_sec=0.01)
            rclpy.spin_once(observer, timeout_sec=0.01)
            if len(values) == len(allocator.thrusters):
                forces = np.array([values[i] for i in range(len(values))])
                if np.allclose(allocator.B @ forces, [0, 0, 4, 0, 0, 0], atol=1e-4):
                    break
        else:
            raise AssertionError('Stamped wrench did not reach motor topics')
        assert max(abs(v) for v in values.values()) <= 10
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(allocator, timeout_sec=0.01)
            rclpy.spin_once(observer, timeout_sec=0.01)
        assert all(v == 0 for v in values.values())
    finally:
        allocator.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
