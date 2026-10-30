"""Build independent RViz display groups without depending on ROS at import time."""

import xml.etree.ElementTree as ET


# Water level is z=0 in lake.sdf. Keep a fixed depth scale for both the
# measured cloud and reference terrain; new hits must not recolor old points.
DEPTH_MIN_Z = -10.0
DEPTH_MAX_Z = 0.0
DEPTH_DEEP_RGB = (32, 80, 160)
DEPTH_SHALLOW_RGB = (185, 225, 255)


def spawn_xy(world_path, model_name='pobs'):
    """Read the initial horizontal offset for display-only estimator alignment."""
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError(f'No world in {world_path}')
    for include in world.findall('include'):
        if include.findtext('name') == model_name:
            pose = [float(value) for value in include.findtext('pose', '0 0 0 0 0 0').split()]
            if len(pose) != 6:
                raise ValueError('Expected six values in robot spawn pose')
            return pose[0], pose[1]
    raise ValueError(f'No included model named {model_name!r} in {world_path}')


def topic(name, transient=False):
    """Use sensor-compatible subscriptions; mapping cloud also supports late joins."""
    return {
        'Value': name,
        'Depth': 1,
        'History Policy': 'Keep Last',
        'Reliability Policy': 'Reliable' if transient else 'Best Effort',
        'Durability Policy': 'Transient Local' if transient else 'Volatile',
    }


def make_config(state_estimation=True, point_cloud_mapping=True, world_frame='lake'):
    """Create both named groups, enabling only the requested displays."""
    return {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'}],
        'Window Geometry': {'Width': 1200, 'Height': 850},
        'Visualization Manager': {
            'Class': '',
            'Global Options': {'Fixed Frame': world_frame, 'Frame Rate': 30},
            # Loading a custom config without Tools leaves the viewport with
            # no active mouse handler, even when its Orbit view is configured.
            # The first tool becomes active when RViz loads the configuration.
            'Tools': [
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Interact'},
                {'Class': 'rviz_default_plugins/Select'},
                {'Class': 'rviz_default_plugins/FocusCamera'},
                {'Class': 'rviz_default_plugins/Measure'},
            ],
            'Displays': [
                {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid',
                 'Enabled': True, 'Cell Size': 5, 'Plane Cell Count': 40, 'Alpha': 0.2},
                {'Class': 'rviz_common/Group', 'Name': 'Lake reference',
                 'Enabled': True, 'Displays': [
                     {'Class': 'rviz_default_plugins/Marker', 'Name': 'Lake terrain',
                      'Enabled': True,
                      'Topic': topic('/pobs/visualization/lake_terrain', transient=True)},
                     {'Class': 'rviz_default_plugins/Marker', 'Name': 'Water surface',
                      'Enabled': False,
                      'Topic': topic('/pobs/visualization/lake_water', transient=True)},
                     {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Sonar test objects',
                      'Enabled': True,
                      'Topic': topic('/pobs/visualization/lake_obstacles', transient=True)},
                 ]},
                {'Class': 'rviz_common/Group', 'Name': 'state_estimation',
                 'Enabled': state_estimation, 'Displays': [
                     {'Class': 'rviz_default_plugins/Path', 'Name': 'Ground truth trail',
                      'Enabled': True,
                      'Topic': topic('/visualization/ground_truth_path', transient=True),
                      'Color': '60; 220; 120', 'Line Style': 'Billboards',
                      'Line Width': 0.06, 'Buffer Length': 1, 'Pose Style': 'None'},
                     {'Class': 'rviz_default_plugins/Path', 'Name': 'Estimated trail',
                      'Enabled': True,
                      'Topic': topic('/visualization/estimated_path', transient=True),
                      'Color': '255; 150; 50', 'Line Style': 'Billboards',
                      'Line Width': 0.04, 'Buffer Length': 1, 'Pose Style': 'None'},
                     {'Class': 'rviz_default_plugins/Pose', 'Name': 'Ground truth pose',
                      'Enabled': True, 'Topic': topic('/model/pobs/pose'),
                      'Shape': 'Arrow', 'Color': '0; 255; 0'},
                 ]},
                {'Class': 'rviz_common/Group', 'Name': 'point_cloud_mapping',
                 'Enabled': point_cloud_mapping, 'Displays': [
                     {'Class': 'rviz_default_plugins/PointCloud2',
                      'Name': 'Accumulated sonar map', 'Enabled': True,
                      'Topic': topic('/pobs/mapping/points', transient=True),
                      'Style': 'Points', 'Size (Pixels)': 5,
                      # Each message contains the complete map; replace the previous
                      # snapshot rather than stacking copies in RViz.
                      'Decay Time': 0, 'Position Transformer': 'XYZ',
                      # RViz's Intensity transformer can use any numeric field;
                      # use world z so this is depth coloring, not echo strength.
                      'Color Transformer': 'Intensity', 'Channel Name': 'z',
                      'Use rainbow': False, 'Autocompute Intensity Bounds': False,
                      'Min Intensity': DEPTH_MIN_Z, 'Max Intensity': DEPTH_MAX_Z,
                      'Min Color': '; '.join(map(str, DEPTH_DEEP_RGB)),
                      'Max Color': '; '.join(map(str, DEPTH_SHALLOW_RGB))},
                     {'Class': 'rviz_default_plugins/Pose', 'Name': 'Ground truth robot pose',
                      'Enabled': True, 'Topic': topic('/pobs/mapping/ground_truth_pose'),
                      'Shape': 'Arrow', 'Color': '0; 255; 0'},
                 ]},
            ],
            'Views': {'Current': {
                'Class': 'rviz_default_plugins/Orbit',
                'Distance': 150 if point_cloud_mapping else 25,
                'Focal Point': {'X': 0, 'Y': 0, 'Z': -3},
                'Pitch': 0.65, 'Yaw': 0.8,
            }},
        },
    }
