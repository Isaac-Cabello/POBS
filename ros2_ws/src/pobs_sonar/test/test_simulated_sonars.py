"""Check valid distance conversion and original simulation timestamps."""

import math

from pobs_sonar.simulated_sonars import range_from_scan
import pytest
from sensor_msgs.msg import LaserScan


def test_range_uses_valid_distance_timestamp_and_explicit_sensor_frame():
    scan = LaserScan()
    scan.header.stamp.sec = 42
    scan.header.stamp.nanosec = 1234
    scan.ranges = [math.inf, 2.5, math.nan, 3.0]
    result = range_from_scan(scan, 'sonar_left_link')
    assert result.range == pytest.approx(2.5)
    assert result.header.stamp == scan.header.stamp
    assert result.header.frame_id == 'sonar_left_link'


@pytest.mark.parametrize('distances', [[], [math.inf], [math.nan], [0.0], [7.0]])
def test_no_return_or_out_of_range_does_not_create_bottom_point(distances):
    scan = LaserScan()
    scan.ranges = distances
    assert range_from_scan(scan, 'sonar_middle_link') is None


@pytest.mark.parametrize('distance', [0.15, 6.1, 20.0, 30.0, math.inf, -math.inf])
def test_downward_sonar_keeps_old_range_limits_and_no_hit_behavior(distance):
    scan = LaserScan()
    scan.header.stamp.sec = 100
    scan.ranges = [distance]
    result = range_from_scan(
        scan, 'sonar_depth_link', min_range=0.15, max_range=30.0,
        field_of_view=0.0, preserve_no_return=True)
    assert result.range == pytest.approx(distance)
    assert result.min_range == pytest.approx(0.15)
    assert result.max_range == 30.0
    assert result.field_of_view == 0.0
    assert result.header.stamp == scan.header.stamp
