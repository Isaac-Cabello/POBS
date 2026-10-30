"""SDF mounting geometry and quaternion utilities, independent of ROS."""

import math
import xml.etree.ElementTree as ET

import numpy as np


def normalized_quaternion(quaternion):
    """Return a finite unit quaternion in ROS xyzw order."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Quaternion must contain four finite values')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Quaternion must have nonzero length')
    return q / norm


def quaternion_matrix(quaternion):
    """Convert ROS xyzw quaternion to a rotation matrix."""
    x, y, z, w = normalized_quaternion(quaternion)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _pose_matrix(pose):
    transform = np.eye(4)
    if pose is None:
        return transform
    if pose.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('Only SDF euler_rpy poses are supported')
    try:
        values = np.array([float(value) for value in (pose.text or '').split()])
    except ValueError as error:
        raise ValueError('SDF pose must contain six finite values') from error
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError('SDF pose must contain six finite values')
    roll, pitch, yaw = values[3:]
    degrees = pose.get('degrees', 'false').lower()
    if degrees not in ('true', 'false', '1', '0'):
        raise ValueError('Invalid SDF pose degrees attribute')
    if degrees in ('true', '1'):
        roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transform[:3, :3] = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ]
    transform[:3, 3] = values[:3]
    return transform


def load_sensor_transforms(sdf_path, sensor_names):
    """Resolve sensor poses relative to the model (not the base link).

    Supports links and named frames in a single model, with explicit relative_to
    references. Reject nested model/include geometry rather than silently
    applying incorrect transforms. The model's world pose is intentionally
    excluded: it comes from the timestamped ground-truth pose stream.
    """
    root = ET.parse(sdf_path).getroot()
    models = [root] if root.tag == 'model' else root.findall('model')
    if len(models) != 1:
        raise ValueError('Expected exactly one top-level SDF model')
    model = models[0]
    if model.find('model') is not None or model.find('include') is not None:
        raise ValueError('Nested models and includes are not supported')
    nodes = {}
    sensors = {}
    model_name = model.get('name')
    if not model_name:
        raise ValueError('SDF model must have a nonempty name')

    def reference(name):
        if name in ('__model__', model_name):
            return '__model__'
        prefix = model_name + '::'
        return name[len(prefix):] if name.startswith(prefix) else name

    def register(name, element, default_parent):
        if not name or name in nodes or name == '__model__':
            raise ValueError(f'Missing or duplicate SDF frame: {name}')
        pose = element.find('pose')
        parent = pose.get('relative_to') if pose is not None else None
        nodes[name] = (reference(parent or default_parent), _pose_matrix(pose))

    for link in model.findall('link'):
        link_name = link.get('name')
        register(link_name, link, '__model__')
        for sensor in link.findall('sensor'):
            sensor_name = sensor.get('name')
            if sensor_name in sensors:
                raise ValueError(f'Ambiguous SDF sensor name: {sensor_name}')
            key = f'{link_name}::{sensor_name}'
            register(key, sensor, link_name)
            sensors[sensor_name] = key
    for frame in model.findall('frame'):
        register(frame.get('name'), frame, frame.get('attached_to', '__model__'))

    resolved = {'__model__': np.eye(4)}

    def resolve(name, visiting):
        if name in resolved:
            return resolved[name]
        if name in visiting:
            raise ValueError(f'Cyclic SDF pose reference: {name}')
        if name not in nodes:
            raise ValueError(f'Unknown or unsupported SDF pose reference: {name}')
        parent, transform = nodes[name]
        resolved[name] = resolve(parent, visiting | {name}) @ transform
        return resolved[name]

    result = {}
    for name in sensor_names:
        if name not in sensors:
            raise ValueError(f'Sensor not found in SDF: {name}')
        result[name] = resolve(sensors[name], set()).copy()
    return result
