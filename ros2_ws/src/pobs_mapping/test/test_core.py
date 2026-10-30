"""Pure mapping tests independent of a running ROS graph."""

import numpy as np
from pobs_mapping.core import PointAccumulator, PoseBuffer
from pobs_mapping.geometry import quaternion_matrix
import pytest


def test_interpolates_translation_and_rotation():
    buffer = PoseBuffer()
    buffer.add(0, [0, 0, 0], [0, 0, 0, 1])
    buffer.add(100, [2, 4, 6], [0, 0, 1, 0])
    position, quaternion = buffer.sample(50)
    np.testing.assert_allclose(position, [1, 2, 3])
    np.testing.assert_allclose(quaternion_matrix(quaternion) @ [1, 0, 0],
                               [0, 1, 0], atol=1e-12)
    assert buffer.sample(-1) is None
    assert buffer.sample(101) is None
    assert buffer.sample(50, max_gap_ns=99) is None
    assert buffer.sample(100, max_gap_ns=99) is not None


def test_out_of_order_duplicate_and_opposite_quaternions():
    buffer = PoseBuffer(max_age_ns=100)
    buffer.add(100, [1, 0, 0], [0, 0, 0, 1])
    buffer.add(0, [0, 0, 0], [0, 0, 0, -1])
    buffer.add(100, [2, 0, 0], [0, 0, 0, 1])
    position, quaternion = buffer.sample(50)
    np.testing.assert_allclose(position, [1, 0, 0])
    np.testing.assert_allclose(quaternion_matrix(quaternion), np.eye(3))
    buffer.add(250, [3, 0, 0], [0, 0, 0, 1])
    assert buffer.oldest_stamp_ns == 100
    assert buffer.newest_stamp_ns == 250
    buffer.clear()
    assert buffer.sample(250) is None
    assert buffer.oldest_stamp_ns is None


def test_projects_mount_then_world_pose_and_retains_all_hits():
    mount = np.eye(4)
    # A downward-facing transducer offset one meter ahead of the model.
    mount[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    mount[:3, 3] = [1, 0, 0]
    cloud = PointAccumulator({'down': mount})
    yaw_90 = [0, 0, np.sqrt(0.5), np.sqrt(0.5)]
    for _ in range(100):
        assert cloud.add('down', 2, 0.1, 30, [10, 20, 30], yaw_90)
    assert len(cloud.points) == 100
    np.testing.assert_allclose(cloud.points, [[10, 21, 28]]*100)
    cloud.clear()
    assert cloud.points == []


@pytest.mark.parametrize('distance', [float('inf'), float('nan'), -1, 0.01, 31])
def test_invalid_measurements_do_not_add_points(distance):
    cloud = PointAccumulator({'s': np.eye(4)})
    assert not cloud.add('s', distance, 0.1, 30, [0, 0, 0], [0, 0, 0, 1])
    assert cloud.points == []
