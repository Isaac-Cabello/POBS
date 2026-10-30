"""Validate repeatable terrain edits, intact shores and closed target meshes."""

from collections import Counter
import importlib.util
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'generate_lake_terrain', PACKAGE.parents[1] / 'scripts/generate_lake_terrain.py')
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)


def test_bumps_preserve_shoreline_and_add_both_hills_and_depressions():
    vertices, _ = terrain.read_obj(PACKAGE / 'models/lake_terrain/meshes/terrain.obj')
    result = terrain.deform_terrain(vertices)
    np.testing.assert_array_equal(result[:, :2], vertices[:, :2])
    shore = vertices[:, 2] >= -0.8
    np.testing.assert_array_equal(result[shore], vertices[shore])
    delta = result[:, 2] - vertices[:, 2]
    assert delta.max() > 0.5
    assert delta.min() < -0.4
    np.testing.assert_array_equal(terrain.deform_terrain(vertices), result)
    assert not np.array_equal(terrain.deform_terrain(vertices, seed=43), result)


def test_regeneration_does_not_compound_bumps_or_change_baseline(tmp_path):
    mesh_folder = tmp_path / 'models/lake_terrain/meshes'
    mesh_folder.mkdir(parents=True)
    baseline = mesh_folder / 'terrain.obj'
    shutil.copyfile(PACKAGE / 'models/lake_terrain/meshes/terrain.obj', baseline)
    original = baseline.read_bytes()
    terrain.generate(tmp_path)
    first = {path.relative_to(tmp_path): path.read_bytes()
             for path in tmp_path.rglob('*') if path.is_file()}
    terrain.generate(tmp_path)
    second = {path.relative_to(tmp_path): path.read_bytes()
              for path in tmp_path.rglob('*') if path.is_file()}
    assert first == second
    assert baseline.read_bytes() == original


def test_boulders_are_closed_with_outward_faces():
    for index in range(1, 4):
        vertices, faces = terrain.read_obj(
            PACKAGE / f'models/sonar_targets/meshes/boulder_{index}.obj')
        edges = Counter(tuple(sorted((a, b))) for face in faces
                        for a, b in zip(face, np.roll(face, -1)))
        assert set(edges.values()) == {2}
        triangles = vertices[faces]
        normals = np.cross(triangles[:, 1] - triangles[:, 0],
                           triangles[:, 2] - triangles[:, 0])
        assert np.all(np.sum(normals * triangles.mean(axis=1), axis=1) > 0)


def test_spawn_has_sonar_clearance_and_targets_match_physics_geometry():
    vertices, faces = terrain.read_obj(PACKAGE / 'models/lake_terrain/meshes/terrain_bumpy.obj')
    world = ET.parse(PACKAGE / 'worlds/lake.sdf').getroot().find('world')
    robot = next(item for item in world.findall('include') if item.findtext('name') == 'pobs')
    x, y, z, *_ = [float(value) for value in robot.findtext('pose').split()]
    assert abs(z - terrain.terrain_height(vertices, faces, x, y) - 2.5) < 1e-5
    model = ET.parse(PACKAGE / 'models/sonar_targets/model.sdf').getroot().find('model')
    assert model.findtext('static') == 'true'
    assert {link.get('name') for link in model.findall('link')} == {
        'boulder_1', 'boulder_2', 'boulder_3', 'wall'}
    for link in model.findall('link'):
        assert ET.canonicalize(ET.tostring(link.find('visual/geometry'), encoding='unicode'),
                               strip_text=True) == ET.canonicalize(
            ET.tostring(link.find('collision/geometry'), encoding='unicode'), strip_text=True)
