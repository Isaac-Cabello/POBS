"""Keep the original launch interface using the selectable RViz implementation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Translate original comparison arguments without duplicating nodes."""
    defaults = {
        'rviz': 'false', 'estimator': 'true', 'publish_odom_transform': 'true',
        'odom_offset_x': '-12.0', 'odom_offset_y': '-8.0', 'reset_settle_s': '0.25',
    }
    target = os.path.join(get_package_share_directory('pobs_visualization'),
                          'launch', 'visualization.launch.py')
    arguments = {
        'state_estimation': 'true', 'point_cloud_mapping': 'false',
        'rviz': LaunchConfiguration('rviz'),
        'start_estimator': LaunchConfiguration('estimator'),
        'publish_odom_transform': LaunchConfiguration('publish_odom_transform'),
        'estimator_offset_x': LaunchConfiguration('odom_offset_x'),
        'estimator_offset_y': LaunchConfiguration('odom_offset_y'),
        'reset_settle_s': LaunchConfiguration('reset_settle_s'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
                                 launch_arguments=arguments.items()),
    ])
