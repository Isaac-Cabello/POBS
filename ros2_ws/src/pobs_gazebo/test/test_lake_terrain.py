"""Validate connected bathymetry, intact shores and optional calibration meshes."""

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


def test_landforms_have_connected_channel_and_significant_relief():
    x = np.linspace(-30, 30, 121)
    axis = -5 + 6 * np.sin((x + 12) / 18)
    channel = terrain.landforms(x, axis)
    shoulders = (terrain.landforms(x, axis - 7)
                 + terrain.landforms(x, axis + 7)) / 2
    assert np.all(shoulders - channel > 1.0)
    vertices, _ = terrain.read_obj(PACKAGE / 'models/lake_terrain/meshes/terrain.obj')
    result = terrain.deform_terrain(vertices)
    delta = result[:, 2] - vertices[:, 2]
    assert np.count_nonzero(delta > 2) > 150
    assert np.count_nonzero(delta < -2) > 150
    assert np.all(result[vertices[:, 2] < -0.8, 2] <= -0.8)
    np.testing.assert_array_equal(terrain.deform_terrain(
        vertices, roughness=0, feature_scale=0), vertices)


def test_regeneration_does_not_compound_bumps_or_change_baseline(tmp_path):
    mesh_folder = tmp_path / 'models/lake_terrain/meshes'
    mesh_folder.mkdir(parents=True)
    baseline = mesh_folder / 'terrain.obj'
    shutil.copyfile(PACKAGE / 'models/lake_terrain/meshes/terrain.obj', baseline)
    shutil.copytree(PACKAGE / 'models/lake_terrain/materials',
                    tmp_path / 'models/lake_terrain/materials')
    original = baseline.read_bytes()
    terrain.generate(tmp_path)
    first = {path.relative_to(tmp_path): path.read_bytes()
             for path in tmp_path.rglob('*') if path.is_file()}
    terrain.generate(tmp_path)
    second = {path.relative_to(tmp_path): path.read_bytes()
              for path in tmp_path.rglob('*') if path.is_file()}
    assert first == second
    assert baseline.read_bytes() == original


def test_elevation_texture_coordinates_follow_height_without_changing_geometry():
    import json
    import struct
    import zlib

    folder = PACKAGE / 'models/lake_terrain'
    original, faces = terrain.read_obj(folder / 'meshes/terrain.obj')
    generated, generated_faces = terrain.read_obj(folder / 'meshes/terrain_bumpy.obj')
    np.testing.assert_allclose(generated, terrain.deform_terrain(original), atol=1e-9)
    np.testing.assert_array_equal(generated_faces, faces)
    uv = np.asarray([[float(value) for value in line.split()[1:]]
                     for line in (folder / 'meshes/terrain_bumpy.obj').read_text().splitlines()
                     if line.startswith('vt ')])
    palette = json.loads((folder / 'materials/elevation_palette.json').read_text())['stops']
    fraction = np.clip((generated[:, 2] - palette[0][0])
                       / (palette[-1][0] - palette[0][0]), 0, 1)
    np.testing.assert_allclose(uv[:, 0], (0.5 + 511 * fraction) / 512, atol=1e-9)
    np.testing.assert_allclose(uv[:, 1], 0.5)
    png = (folder / 'materials/elevation.png').read_bytes()
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
    width, height = struct.unpack('!II', png[16:24])
    assert (width, height) == (512, 8)
    size = struct.unpack('!I', png[33:37])[0]
    raw = zlib.decompress(png[41:41 + size])
    assert raw[1:4] == bytes(palette[0][1:])
    assert raw[1 + 511 * 3:1 + 512 * 3] == bytes(palette[-1][1:])
    model = ET.parse(folder / 'model.sdf').getroot()
    texture = model.findtext('.//visual[@name="terrain_visual"]/material/pbr/metal/albedo_map')
    assert texture == 'model://lake_terrain/materials/elevation.png'


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
    for filename in ('lake.sdf', 'lake_cad.sdf'):
        world = ET.parse(PACKAGE / 'worlds' / filename).getroot().find('world')
        assert all(item.findtext('uri') != 'model://sonar_targets'
                   for item in world.findall('include'))
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
