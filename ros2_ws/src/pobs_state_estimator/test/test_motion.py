"""Regression checks for the earlier IMU x/y and independent water-depth input."""

import math

from geometry_msgs.msg import PointStamped
from pobs_state_estimator.state_estimator import StateEstimator
import pytest
import rclpy
from sensor_msgs.msg import Imu


@pytest.fixture
def estimator():
    rclpy.init()
    node = StateEstimator()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def sample(sec, ax=1.0, yaw=0.0):
    message = Imu()
    message.header.stamp.sec = sec
    message.orientation.z = math.sin(yaw / 2)
    message.orientation.w = math.cos(yaw / 2)
    message.linear_acceleration.x = ax
    message.linear_acceleration.z = 9.81
    return message


def test_acceleration_moves_horizontal_estimate_using_acquisition_time(estimator):
    estimator.imu_callback(sample(1))
    estimator.imu_callback(sample(2))
    estimator.imu_callback(sample(3))
    assert estimator.position_x == pytest.approx(2.0)
    assert estimator.velocity_x == pytest.approx(2.0)
    assert estimator.position_y == pytest.approx(0.0)


def test_body_acceleration_rotates_into_world_and_reset_clears_xy(estimator):
    estimator.imu_callback(sample(10, yaw=math.pi / 2))
    estimator.imu_callback(sample(11, yaw=math.pi / 2))
    assert estimator.position_x == pytest.approx(0, abs=1e-9)
    assert estimator.position_y == pytest.approx(0.5)
    estimator.imu_callback(sample(11, yaw=math.pi / 2))
    assert estimator.position_y == pytest.approx(0.5)
    estimator.imu_callback(sample(1))
    assert estimator.position_x == estimator.position_y == 0
    assert estimator.velocity_x == estimator.velocity_y == 0


def test_depth_input_retains_absolute_water_depth_semantics(estimator):
    depth = PointStamped()
    depth.header.stamp.sec = 1
    depth.point.z = 3.2
    estimator.depth_callback(depth)
    assert estimator.depth_m == pytest.approx(3.2)
    assert estimator.depth_topic == '/pobs/depth'
