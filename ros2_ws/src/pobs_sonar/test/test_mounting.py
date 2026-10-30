"""Check that ROS mounting transforms and Gazebo sensor geometry agree."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from pobs_sonar.mounting import SONAR_MOUNTS
import pytest


def test_mounted_sensor_poses_and_existing_downward_sensor():
    model_path = (Path(__file__).parents[2] / 'pobs_gazebo' / 'models'
                  / 'pobs' / 'model.sdf')
    root = ET.parse(model_path).getroot()
    base = root.find('model/link[@name="base_link"]')
    for position in ('left', 'right', 'middle'):
        sensor = base.find(f'sensor[@name="sonar_{position}"]')
        pose = [float(value) for value in sensor.findtext('pose').split()]
        assert pose == pytest.approx(SONAR_MOUNTS[position])
        assert sensor.findtext('topic') == f'/pobs/sonar/sonar_{position}/scan'
        assert float(sensor.findtext('lidar/range/min')) == 0.05
        assert float(sensor.findtext('lidar/range/max')) == 6.0
        assert base.find(f'visual[@name="sonar_{position}_housing"]') is not None
    depth = base.find('sensor[@name="sonar_depth"]')
    assert depth.findtext('topic') == '/pobs/sonar/sonar_depth/scan'
    assert [float(value) for value in depth.findtext('pose').split()] == (
        pytest.approx(SONAR_MOUNTS['depth']))
    assert float(depth.findtext('lidar/range/min')) == 0.15
    assert float(depth.findtext('lidar/range/max')) == 30.0
    assert float(depth.findtext('update_rate')) == 10.0
    assert float(depth.findtext('lidar/range/resolution')) == 0.01
    assert float(depth.findtext('lidar/noise/stddev')) == 0.01
    assert int(depth.findtext('lidar/scan/horizontal/samples')) == 1


def test_front_sensors_point_forward_and_down_without_sideways_rotation():
    for position in ('left', 'right'):
        x, y, z, roll, pitch, yaw = SONAR_MOUNTS[position]
        assert x > 0.4  # Outside the 0.8 m chassis, away from self-returns.
        assert z < 0
        assert roll == yaw == 0.0
        assert math.cos(pitch) == pytest.approx(math.sqrt(0.5))
        assert -math.sin(pitch) == pytest.approx(-math.sqrt(0.5))
    assert SONAR_MOUNTS['left'][1] > 0
    assert SONAR_MOUNTS['right'][1] < 0
    assert SONAR_MOUNTS['middle'][3:] == (0.0, 0.0, 0.0)
