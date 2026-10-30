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
    mapping = _enabled(context, 'point_cloud_mapping')
    sim_time = _enabled(context, 'use_sim_time')
    frame = LaunchConfiguration('world_frame').perform(context)
    world_path = LaunchConfiguration('world_path').perform(context)
    show_rviz = _enabled(context, 'rviz')
    spawn_x, spawn_y = -12.0, -8.0
    if state or (show_rviz and os.path.isfile(world_path)):
        spawn_x, spawn_y = spawn_xy(world_path)
    actions = [Node(
        package='pobs_visualization', executable='lake_reference_scene',
        name='lake_reference_scene', output='screen',
        parameters=[{'world_path': world_path, 'world_frame': frame,
                     'use_sim_time': sim_time}],
    )]
    if sim_time and _enabled(context, 'bridge_clock'):
        world_name = LaunchConfiguration('world_name').perform(context)
        actions.append(ExecuteProcess(
            cmd=[sys.executable, '-m', 'pobs_visualization.clock_bridge', world_name],
            name='visualization_clock_bridge', output='screen',
        ))
    if state:
        x = LaunchConfiguration('estimator_offset_x').perform(context) or str(spawn_x)
        y = LaunchConfiguration('estimator_offset_y').perform(context) or str(spawn_y)
        actions.append(Node(
            package='pobs_visualization', executable='trajectory_comparison',
            name='trajectory_comparison', output='screen',
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
                parameters=[{'use_sim_time': sim_time}], output='screen',
            ))
    if mapping:
        actions.append(Node(
            package='pobs_mapping', executable='point_cloud_mapper',
            name='point_cloud_mapper', output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path').perform(context),
                'world_frame': frame, 'use_sim_time': sim_time,
            }],
        ))
    if show_rviz:
        config = make_config(state, mapping, frame)
        if state:
            config['Visualization Manager']['Views']['Current']['Distance'] = 30 if mapping else 25
            config['Visualization Manager']['Views']['Current']['Focal Point'].update(
                {'X': spawn_x, 'Y': spawn_y})
        handle, path = tempfile.mkstemp(prefix='pobs-', suffix='.rviz')
        with os.fdopen(handle, 'w') as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        # RViz reads this file on startup; remove it only when launch shuts down.
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=_remove_config, args=[path])
        ])))
        actions.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', path],
            parameters=[{'use_sim_time': sim_time}], output='screen',
        ))
    return actions


def generate_launch_description():
    """Expose independent display toggles and geometry paths."""
    workspace = os.environ.get('POBS_WORKSPACE', os.path.expanduser('~/ros2_ws'))
    gazebo = os.path.join(workspace, 'src', 'pobs_gazebo')
    arguments = {
        'state_estimation': 'true', 'point_cloud_mapping': 'true',
        'rviz': 'true', 'start_estimator': 'true', 'use_sim_time': 'true',
        # The sensor bridge owns /clock. Enable only when running without it.
        'bridge_clock': 'false',
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
