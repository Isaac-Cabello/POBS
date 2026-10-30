"""ROS integration checks for persistent mapping, using controlled truth inputs."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


def stamp(message, seconds):
    message.sec = int(seconds)
    message.nanosec = int(round((seconds - int(seconds)) * 1e9))


rclpy.init()
node = Node('mapping_integration_check')
clouds = []
normalized = []
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription = node.create_subscription(
    PointCloud2, '/pobs/mapping/points', clouds.append, latched)
pose_sub = node.create_subscription(
    PoseStamped, '/pobs/mapping/ground_truth_pose', normalized.append, 10)
pose_pub = node.create_publisher(PoseStamped, '/model/pobs/pose', 10)
odom_pub = node.create_publisher(Odometry, '/state/odometry', 10)
clock_pub = node.create_publisher(Clock, '/clock', 10)
range_pubs = {name: node.create_publisher(
    Range, f'/pobs/sonar/sonar_{name}/range', 10)
    for name in ('depth', 'left', 'right', 'middle')}
log = open('/tmp/mapper_ros_test.log', 'w+')
process = subprocess.Popen([
    'ros2', 'run', 'pobs_mapping', 'point_cloud_mapper', '--ros-args',
    '-p', 'model_path:=/repo/ros2_ws/src/pobs_gazebo/models/pobs/model.sdf',
    '-p', 'world_frame:=lake', '-p', 'use_sim_time:=false'], stdout=log, stderr=log, start_new_session=True)


def spin_until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Mapper exited early'
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('Timed out waiting for mapper condition')


def pump(duration=0.25):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)


def same_points(actual, expected):
    # Independent ROS topics have no guaranteed cross-topic callback order.
    return all(any(np.allclose(point, candidate, atol=1e-5)
                   for candidate in actual) for point in expected)


def pose(seconds, x, yaw=0):
    msg = PoseStamped()
    stamp(msg.header.stamp, seconds)
    msg.header.frame_id = 'lake'
    msg.pose.position.x = float(x)
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        pose_pub.publish(msg)
        pump(0.1)
        if normalized and normalized[-1].header.stamp == msg.header.stamp:
            return
    raise AssertionError('Ground truth was not received')


def ranges(seconds, distance=1.0):
    for name, publisher in range_pubs.items():
        msg = Range()
        stamp(msg.header.stamp, seconds)
        msg.header.frame_id = f'sonar_{name}_link'
        msg.min_range, msg.max_range = 0.05, 30.0
        msg.range = float(distance)
        publisher.publish(msg)
    pump()


try:
    spin_until(lambda: all(pub.get_subscription_count() for pub in range_pubs.values())
               and pose_pub.get_subscription_count())
    pose(1, 0)
    ranges(1)
    spin_until(lambda: clouds and clouds[-1].width == 4)
    first = np.array([list(row) for row in point_cloud2.read_points(
        clouds[-1], field_names=('x', 'y', 'z'))])
    pose(1.2, 2)
    # Interpolate both translation and orientation using historical truth.
    ranges(1.1)
    spin_until(lambda: clouds[-1].width == 8)
    second = np.array([list(row) for row in point_cloud2.read_points(
        clouds[-1], field_names=('x', 'y', 'z'))])
    assert same_points(second[4:], first + [1, 0, 0]), second
    pose(1.4, 3, math.pi / 2)
    ranges(1.4)
    spin_until(lambda: clouds[-1].width == 12)
    third = np.array([list(row) for row in point_cloud2.read_points(
        clouds[-1], field_names=('x', 'y', 'z'))])
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert same_points(third[8:], first @ rotation.T + [3, 0, 0]), third
    assert clouds[-1].header.frame_id == 'lake'
    assert normalized and normalized[-1].header.frame_id == 'lake'
    late_clouds = []
    late_sub = node.create_subscription(PointCloud2, '/pobs/mapping/points',
                                        late_clouds.append, latched)
    spin_until(lambda: late_clouds and late_clouds[-1].width == 12)
    for value in (float('inf'), float('nan'), -1., 31.):
        ranges(1.4, value)
    odom = Odometry()
    odom.pose.pose.position.x = 1000.
    odom.pose.pose.orientation.w = 1.
    odom_pub.publish(odom)
    ranges(1.4)
    spin_until(lambda: clouds[-1].width == 16)
    current = list(point_cloud2.read_points(clouds[-1], field_names=('x',)))
    assert max(float(row[0]) for row in current) < 10, current
    clear = node.create_client(Trigger, '/pobs/mapping/clear')
    assert clear.wait_for_service(timeout_sec=5)
    future = clear.call_async(Trigger.Request())
    spin_until(lambda: future.done() and clouds[-1].width == 0)
    pose(2, 0)
    ranges(2)
    spin_until(lambda: clouds[-1].width == 4)
    clock = Clock()
    stamp(clock.clock, 20)
    clock_pub.publish(clock)
    pump()
    stamp(clock.clock, 0)
    clock_pub.publish(clock)
    spin_until(lambda: clouds[-1].width == 0)
    ranges(2)
    pump()
    assert clouds[-1].width == 0, 'Old poses survived reset'
    print('PASS: >4 persistent points, timestamp interpolation, rotated robot, '
          'latched full map, invalid-range rejection, estimator isolation, '
          'manual clear, /clock rollback and old-pose rejection')
except BaseException:
    log.flush()
    print(Path('/tmp/mapper_ros_test.log').read_text()[-18000:])
    raise
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
    node.destroy_node()
    rclpy.try_shutdown()


