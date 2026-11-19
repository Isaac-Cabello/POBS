"""Exercise sampled-beam geometry and per-cell overlap rules."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pobs_mapping.elevation_color import elevation_rgb
from pobs_mapping.footprint import BEAM_HALF_ANGLE, BEAM_SLOPE, FootprintMap, beam_footprint
from pobs_mapping.geometry import load_sensor_transforms
import pytest


LEVEL = [0, 0, 0, 1]
MODEL = Path(__file__).resolve().parents[2] / 'pobs_gazebo' / 'models' / 'pobs' / 'model.sdf'


def test_highest_hit_uses_world_elevation_and_centers_circle_on_beam_axis():
    mount = np.eye(4)
    mount[:3, 3] = [0.5, 0, 0]
    angle = math.radians(45)
    attitude = [0, math.sin(angle / 2), 0, math.cos(angle / 2)]
    # The second hit is farther away but higher after applying the robot pitch.
    # The last two entries represent a ray outside the beam and a missed ray.
    hits = np.array([[2, 0, 0], [2.2, 0, 0.3], [1, 0.13, 0.13], [math.inf, 0, 0]])
    x, y, radius, elevation = beam_footprint(hits, mount, [1, 2, -3], attitude)
    distance = math.hypot(2.2, 0.3)
    assert x == pytest.approx(1 + (0.5 + distance) * math.cos(angle))
    assert y == pytest.approx(2)
    assert radius == pytest.approx(distance * BEAM_SLOPE)
    assert elevation == pytest.approx(-3 - 2.7 * math.sin(angle) + 0.3 * math.cos(angle))


def test_newer_patch_replaces_only_overlap_even_at_a_lower_elevation():
    grid = FootprintMap({}, 0.1)
    assert grid.paint(0.05, 0.05, 0.11, -2.0, 10)
    assert grid.paint(0.15, 0.05, 0.11, -4.0, 20)
    assert grid.cells[(-1, 0)] == (10, -2.0)
    assert grid.cells[(0, 0)] == (20, -4.0)
    assert grid.cells[(1, 0)] == (20, -4.0)
    assert grid.cells[(2, 0)] == (20, -4.0)
    assert grid.cells[(0, 1)] == (10, -2.0)
    assert (2, 1) not in grid.cells


@pytest.mark.parametrize('readings', [
    [(20, -2.0), (20, -4.0), (10, -6.0)],
    [(10, -6.0), (20, -4.0), (20, -2.0)],
])
def test_delayed_coverage_and_equal_timestamp_elevation_tie(readings):
    grid = FootprintMap({}, 0.1)
    for stamp, height in readings:
        radius = 0.21 if stamp == 10 else 0.11
        grid.paint(0.05, 0.05, radius, height, stamp)
    assert grid.cells[(0, 0)] == (20, -4.0)
    assert grid.cells[(-2, 0)] == (10, -6.0)
    grid.clear()
    assert list(grid.elevations()) == []


def test_no_hit_leaves_map_empty():
    grid = FootprintMap({'middle': np.eye(4)}, 0.1)
    assert not grid.add('middle', np.array([[math.nan, 0, 0]]), [0, 0, 0], LEVEL, 1)
    assert not grid.cells


def test_mapping_a_new_area_preserves_the_previous_surface():
    grid = FootprintMap({'middle': np.eye(4)}, 0.1)
    hits = np.array([[1, 0, 0]])
    assert grid.add('middle', hits, [0, 0, -4], LEVEL, 10)
    first_surface = set(grid.elevations())
    assert first_surface
    assert grid.add('middle', hits, [5, 0, -7], LEVEL, 20)
    surface = set(grid.elevations())
    assert first_surface < surface
    assert {tile[2] for tile in surface} == {-4.0, -7.0}


def test_unchanged_tile_refreshes_timestamp_without_redrawing():
    grid = FootprintMap({}, 0.1)
    assert grid.paint(0.05, 0.05, 0.01, -4, 10)
    assert not grid.paint(0.05, 0.05, 0.01, -4, 30)
    assert not grid.paint(0.05, 0.05, 0.01, -7, 20)
    assert grid.cells == {(0, 0): (30, -4)}


def test_elevation_colors_stay_fixed_as_coverage_grows():
    shallow, deep = elevation_rgb(0), elevation_rgb(-10)
    assert elevation_rgb(3) == shallow
    assert elevation_rgb(-14) == deep
    assert elevation_rgb(-5) == pytest.approx((40 / 255, 90 / 255, 194 / 255))


@pytest.mark.parametrize('position', ['left', 'right', 'middle', 'depth'])
def test_gazebo_beam_spans_sixteen_degrees_at_legacy_sensor_mount(position):
    name = f'sonar_{position}'
    beam = name + '_beam'
    transforms = load_sensor_transforms(MODEL, [name, beam])
    np.testing.assert_allclose(transforms[name], transforms[beam])
    sensor = ET.parse(MODEL).find(f"model/link/sensor[@name='{beam}']")
    assert sensor.findtext('topic') == f'/pobs/sonar/{name}/beam'
    for axis in ('horizontal', 'vertical'):
        scan = sensor.find(f'lidar/scan/{axis}')
        assert scan.findtext('samples') == '17'
        assert float(scan.findtext('min_angle')) == pytest.approx(-BEAM_HALF_ANGLE)
        assert float(scan.findtext('max_angle')) == pytest.approx(BEAM_HALF_ANGLE)
