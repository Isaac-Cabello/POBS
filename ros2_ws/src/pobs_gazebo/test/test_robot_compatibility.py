"""Keep optional visuals and renamed motors consistent with the current robot."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
MOTORS = ('front_left', 'front_right', 'back_left', 'back_right',
          'left_thruster', 'right_thruster')


def test_descriptive_motor_commands_have_real_joint_and_plugin_targets():
    model = ET.parse(PACKAGE / 'models/pobs/model.sdf').getroot().find('model')
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    plugin_joints = {plugin.findtext('joint_name') for plugin in model.findall('plugin')
                     if plugin.get('name') == 'gz::sim::systems::Thruster'}
    for name in MOTORS:
        assert model.find(f"link[@name='{name}']") is not None
        assert joints[f'{name}_joint'].findtext('child') == name
        assert f'{name}_joint' in plugin_joints


def test_cad_changes_only_chassis_visual_and_keeps_current_sonar_and_physics():
    simple = ET.parse(PACKAGE / 'models/pobs/model.sdf').getroot()
    cad = ET.parse(PACKAGE / 'models/pobs_cad/model.sdf').getroot()
    for root in (simple, cad):
        model = root.find('model')
        model.set('name', 'pobs')
        body = model.find("link[@name='base_link']")
        body.remove(body.find("visual[@name='visual']"))
    assert ET.canonicalize(ET.tostring(simple, encoding='unicode'), strip_text=True) == \
        ET.canonicalize(ET.tostring(cad, encoding='unicode'), strip_text=True)


def test_cad_world_keeps_new_terrain_targets_and_spawn():
    worlds = [ET.parse(PACKAGE / f'worlds/{name}.sdf').getroot()
              for name in ('lake', 'lake_cad')]
    for root in worlds:
        for include in root.findall('world/include'):
            if include.findtext('name') == 'pobs':
                include.find('uri').text = 'model://pobs'
    assert ET.canonicalize(ET.tostring(worlds[0], encoding='unicode'), strip_text=True) == \
        ET.canonicalize(ET.tostring(worlds[1], encoding='unicode'), strip_text=True)
