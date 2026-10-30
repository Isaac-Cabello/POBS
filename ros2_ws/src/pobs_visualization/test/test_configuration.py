"""Check frame alignment and persistent RViz cloud configuration."""

from pathlib import Path

from pobs_visualization.configuration import make_config, spawn_xy
import pytest


def test_groups_independently_toggle_and_use_truth_for_mapping():
    manager = make_config(False, True)['Visualization Manager']
    groups = {display['Name']: display for display in manager['Displays']}
    assert not groups['state_estimation']['Enabled']
    mapping = groups['point_cloud_mapping']
    assert mapping['Enabled']
    assert manager['Global Options']['Fixed Frame'] == 'lake'
    cloud, pose = mapping['Displays']
    assert cloud['Topic']['Value'] == '/pobs/mapping/points'
    assert cloud['Topic']['Durability Policy'] == 'Transient Local'
    assert cloud['Decay Time'] == 0
    assert cloud['Color Transformer'] == 'Intensity'
    assert cloud['Channel Name'] == 'z'
    assert not cloud['Use rainbow']
    assert not cloud['Autocompute Intensity Bounds']
    assert cloud['Min Intensity'] == -10.0
    assert cloud['Max Intensity'] == 0.0
    assert pose['Topic']['Value'] == '/pobs/mapping/ground_truth_pose'
    reference = groups['Lake reference']
    assert reference['Enabled']
    terrain, water, objects = reference['Displays']
    assert terrain['Enabled']
    assert terrain['Topic']['Durability Policy'] == 'Transient Local'
    assert not water['Enabled']
    assert objects['Enabled']
    assert objects['Topic']['Value'] == '/pobs/visualization/lake_obstacles'
    view = manager['Views']['Current']
    assert view['Distance'] >= 100
    assert view['Focal Point']['X'] == 0


def test_spawn_alignment_uses_horizontal_origin_only(tmp_path):
    world = tmp_path / 'lake.sdf'
    world.write_text("""<sdf><world name="lake"><include><name>pobs</name>
        <pose>-12 -8 -3 0 0 0</pose></include></world></sdf>""")
    assert spawn_xy(world) == (-12, -8)


@pytest.mark.parametrize('state,mapping', [(True, False), (False, True), (True, True)])
def test_each_view_starts_with_mouse_camera_controls(state, mapping):
    manager = make_config(state, mapping)['Visualization Manager']
    tools = [tool['Class'] for tool in manager['Tools']]
    assert tools[0] == 'rviz_default_plugins/MoveCamera'
    assert 'rviz_default_plugins/Interact' in tools
    assert 'rviz_default_plugins/FocusCamera' in tools


def test_spawn_alignment_rejects_missing_robot(tmp_path):
    world = tmp_path / 'lake.sdf'
    world.write_text('<sdf><world name="lake" /></sdf>')
    with pytest.raises(ValueError, match='No included model'):
        spawn_xy(world)


def test_repository_spawn():
    source = Path(__file__).resolve().parents[2]
    world = source / 'pobs_gazebo' / 'worlds' / 'lake.sdf'
    if not world.exists():
        pytest.skip('Source Gazebo package not installed beside visualization')
    assert spawn_xy(world) == (-12, -8)


def test_state_view_restores_green_truth_and_orange_estimated_trails():
    manager = make_config(True, False)['Visualization Manager']
    group = next(display for display in manager['Displays']
                 if display['Name'] == 'state_estimation')
    truth, estimate, pose = group['Displays']
    assert truth['Topic']['Value'] == '/visualization/ground_truth_path'
    assert truth['Color'] == '60; 220; 120'
    assert estimate['Topic']['Value'] == '/visualization/estimated_path'
    assert estimate['Color'] == '255; 150; 50'
    assert pose['Topic']['Value'] == '/model/pobs/pose'
