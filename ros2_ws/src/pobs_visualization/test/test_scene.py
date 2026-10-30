"""Check that reference geometry stays aligned with the Gazebo world."""

from pathlib import Path

import numpy as np
from pobs_visualization.scene import lake_geometry, obj_triangles
import pytest


def test_obj_faces_use_vertex_indices_and_triangulate_polygons(tmp_path):
    mesh = tmp_path / 'quad.obj'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n'
                    'f -4/1/1 -3/2/1 -2/3/1 -1/4/1\n')
    np.testing.assert_allclose(obj_triangles(mesh), [
        [0, 0, 0], [1, 0, 0], [1, 1, 0],
        [0, 0, 0], [1, 1, 0], [0, 1, 0],
    ])


def test_geometry_combines_world_pose_link_pose_visual_pose_and_mesh_scale(tmp_path):
    worlds = tmp_path / 'worlds'
    models = tmp_path / 'models' / 'lake_terrain'
    worlds.mkdir()
    models.mkdir(parents=True)
    (models / 'mesh.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
    (models / 'model.sdf').write_text("""<sdf><model name="lake_terrain">
      <link name="terrain"><pose>0 0 3 0 0 0</pose>
        <visual name="terrain_visual"><pose>1 0 0 0 0 0</pose><geometry><mesh>
          <uri>model://lake_terrain/mesh.obj</uri><scale>2 2 2</scale>
        </mesh></geometry></visual></link></model></sdf>""")
    world = worlds / 'test.sdf'
    world.write_text("""<sdf><world name="test"><include>
      <uri>model://lake_terrain</uri><pose degrees="true">10 20 0 0 0 90</pose>
    </include></world></sdf>""")
    np.testing.assert_allclose(lake_geometry(world)['terrain_visual'], [
        [10, 21, 3], [10, 23, 3], [8, 21, 3],
    ])


def test_repository_terrain_contains_real_lake_depth_and_water_level():
    world = Path(__file__).resolve().parents[2] / 'pobs_gazebo' / 'worlds' / 'lake.sdf'
    if not world.exists():
        pytest.skip('Source Gazebo package not installed beside visualization')
    geometry = lake_geometry(world)
    assert geometry['terrain_visual'].shape[0] > 1000
    assert geometry['terrain_visual'][:, 2].min() < -5
    np.testing.assert_allclose(geometry['water_surface'][:, 2], 0)
    objects = {name: points for name, points in geometry.items()
               if name.startswith('sonar_targets/')}
    assert len(objects) == 4
    wall = objects['sonar_targets/wall/wall']
    np.testing.assert_allclose(wall.max(axis=0) - wall.min(axis=0), [0.4, 6, 4.5])
    np.testing.assert_allclose(wall[:, 0].min(), -7.0)
