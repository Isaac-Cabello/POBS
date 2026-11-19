"""Launch selected display groups and the ground-truth sonar mapper."""

import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from pobs_sonar.mounting import SONAR_MOUNTS
from pobs_visualization.configuration import make_config, spawn_xy

import yaml


def _enabled(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false')
    return value in ('true', '1')


def _remove_config(context, path):
    """Remove the generated configuration during launch shutdown."""
    del context
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    return []


def _launch(context):
    state = _enabled(context, 'state_estimation')
    footprints = _enabled(context, 'footprint_mapping')
    points = _enabled(context, 'point_cloud_mapping') and not footprints
    sim_time = _enabled(context, 'use_sim_time')
    # The sensor launcher also bridges /clock. Use one Gazebo clock for this
    # view so interleaved deliveries cannot appear to rewind simulation time.
    clock_topic = '/pobs/visualization/clock'
    clock_remaps = [('/clock', clock_topic)] if sim_time else []
    frame = LaunchConfiguration('world_frame').perform(context)
    world_path = LaunchConfiguration('world_path').perform(context)
    show_rviz = _enabled(context, 'rviz')
    spawn_x, spawn_y = -12.0, -8.0
    if state or (show_rviz and os.path.isfile(world_path)):
        spawn_x, spawn_y = spawn_xy(world_path)
    actions = [Node(
        package='pobs_visualization', executable='lake_reference_scene',
        name='lake_reference_scene', output='screen',
        remappings=clock_remaps,
        parameters=[{'world_path': world_path, 'world_frame': frame,
                     'use_sim_time': sim_time}],
    )]
    if sim_time:
        world_name = LaunchConfiguration('world_name').perform(context)
        actions.append(ExecuteProcess(
            cmd=[sys.executable, '-m', 'pobs_visualization.clock_bridge', world_name, clock_topic],
            name='visualization_clock_bridge', output='screen',
        ))
    if state:
        x = LaunchConfiguration('estimator_offset_x').perform(context) or str(spawn_x)
        y = LaunchConfiguration('estimator_offset_y').perform(context) or str(spawn_y)
        actions.append(Node(
            package='pobs_visualization', executable='trajectory_comparison',
            name='trajectory_comparison', output='screen',
            remappings=clock_remaps,
            parameters=[{
                'use_sim_time': sim_time, 'world_frame': frame,
                'publish_lake': False,
                'publish_odom_transform': _enabled(context, 'publish_odom_transform'),
                'odom_offset_x': float(x), 'odom_offset_y': float(y),
                'reset_settle_s': float(LaunchConfiguration('reset_settle_s').perform(context)),
            }],
        ))
        if _enabled(context, 'start_estimator'):
            actions.append(Node(
                package='pobs_state_estimator', executable='state_estimator',
                parameters=[{'use_sim_time': sim_time}], remappings=clock_remaps, output='screen',
            ))
    if footprints:
        actions.append(ExecuteProcess(
            name='sonar_beam_bridge',
            cmd=[sys.executable, '-m', 'pobs_sonar.managed_bridge', *[
                f'/pobs/sonar/sonar_{position}/beam/points'
                '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
                for position in SONAR_MOUNTS
            ], '--ros-args', '-r', '__node:=sonar_beam_bridge'], output='screen',
        ))
    if points or footprints:
        mapper_parameters = {
            'model_path': LaunchConfiguration('model_path').perform(context),
            'world_frame': frame, 'use_sim_time': sim_time,
            'mapping_method': 'footprints' if footprints else 'points',
        }
        cell_size = LaunchConfiguration('footprint_cell_size_m').perform(context)
        if cell_size:
            mapper_parameters['footprint_cell_size_m'] = float(cell_size)
        actions.append(Node(
            package='pobs_mapping', executable='point_cloud_mapper',
            name='point_cloud_mapper', output='screen',
            remappings=clock_remaps,
            parameters=[mapper_parameters],
        ))
    if show_rviz:
        config = make_config(state, points, frame, footprint_mapping=footprints)
        if state or footprints:
            view = config['Visualization Manager']['Views']['Current']
            view['Distance'] = 30 if points or footprints else 25
            view['Focal Point'].update({'X': spawn_x, 'Y': spawn_y})
        handle, path = tempfile.mkstemp(prefix='pobs-', suffix='.rviz')
        with os.fdopen(handle, 'w') as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        # RViz reads this file on startup; remove it only when launch shuts down.
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=_remove_config, args=[path])
        ])))
        actions.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', path],
            parameters=[{'use_sim_time': sim_time}], remappings=clock_remaps, output='screen',
        ))
    return actions


def generate_launch_description():
    """Expose independent display toggles and geometry paths."""
    workspace = os.environ.get('POBS_WORKSPACE', os.path.expanduser('~/ros2_ws'))
    gazebo = os.path.join(workspace, 'src', 'pobs_gazebo')
    arguments = {
        'state_estimation': 'true', 'point_cloud_mapping': 'true',
        'footprint_mapping': 'false', 'footprint_cell_size_m': '',
        'rviz': 'true', 'start_estimator': 'true', 'use_sim_time': 'true',
        'publish_odom_transform': 'true', 'reset_settle_s': '0.25',
        'world_frame': 'lake', 'world_name': 'lake',
        'model_path': os.path.join(gazebo, 'models', 'pobs', 'model.sdf'),
        'world_path': os.path.join(gazebo, 'worlds', 'lake.sdf'),
        'estimator_offset_x': '', 'estimator_offset_y': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in arguments.items()],
        OpaqueFunction(function=_launch),
    ])
