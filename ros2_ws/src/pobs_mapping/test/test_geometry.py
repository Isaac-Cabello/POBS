"""Exercise SDF frame semantics and sonar beam projection geometry."""

import numpy as np
from pobs_mapping.geometry import load_sensor_transforms, quaternion_matrix
import pytest


def write_sdf(tmp_path, contents):
    path = tmp_path / 'model.sdf'
    path.write_text(contents)
    return path


def test_resolves_link_frame_offset_and_rotation(tmp_path):
    path = write_sdf(tmp_path, """<sdf><model name="robot">
      <pose>100 200 300 0 0 0</pose>
      <link name="base"><pose degrees="true">1 0 0 0 0 90</pose>
        <sensor name="front"><pose>2 0 0 0 0 0</pose></sensor>
        <sensor name="down"><pose relative_to="mount" degrees="true">
          0 0 0 0 90 0</pose></sensor></link>
      <frame name="mount" attached_to="base"><pose>0 1 0 0 0 0</pose></frame>
    </model></sdf>""")
    transforms = load_sensor_transforms(path, ['front', 'down'])
    np.testing.assert_allclose(transforms['front'][:3, 3], [1, 2, 0], atol=1e-12)
    np.testing.assert_allclose(transforms['down'][:3, 3], [0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(transforms['down'][:3, 0], [0, 0, -1], atol=1e-12)


@pytest.mark.parametrize('contents, message', [
    ('<link name="base"><sensor name="s"><pose relative_to="missing">'
     '0 0 0 0 0 0</pose></sensor></link>', 'Unknown'),
    ('<link name="base"><sensor name="s"><pose rotation_format="quat_xyzw">'
     '0 0 0 0 0 0 1</pose></sensor></link>', 'euler_rpy'),
    ('<link name="base"><sensor name="s"><pose>0 0</pose></sensor></link>',
     'six finite'),
])
def test_rejects_unsupported_or_invalid_sdf(tmp_path, contents, message):
    path = write_sdf(tmp_path, '<sdf><model name="robot">' + contents +
                     '</model></sdf>')
    with pytest.raises(ValueError, match=message):
        load_sensor_transforms(path, ['s'])


def test_rejects_cyclic_pose_chain(tmp_path):
    path = write_sdf(tmp_path, """<sdf><model name="robot">
      <frame name="a" attached_to="b"/><frame name="b" attached_to="a"/>
      <link name="base"><pose relative_to="a">0 0 0 0 0 0</pose>
      <sensor name="s"/></link></model></sdf>""")
    with pytest.raises(ValueError, match='Cyclic'):
        load_sensor_transforms(path, ['s'])


def test_quaternion_normalizes_and_rejects_zero():
    np.testing.assert_allclose(quaternion_matrix([0, 0, 0, 2]), np.eye(3))
    with pytest.raises(ValueError, match='nonzero'):
        quaternion_matrix([0, 0, 0, 0])


def test_rejects_missing_model_name(tmp_path):
    path = write_sdf(tmp_path, '<sdf><model><link name="base"/></model></sdf>')
    with pytest.raises(ValueError, match='nonempty name'):
        load_sensor_transforms(path, ['s'])
