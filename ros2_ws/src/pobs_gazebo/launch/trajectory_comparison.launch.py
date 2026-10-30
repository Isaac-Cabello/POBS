<<<<<<< HEAD
"""Keep the original launch interface using the selectable RViz implementation."""
=======
"""Launch trajectory comparison and optionally its configured RViz view."""
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
<<<<<<< HEAD
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
=======
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Return the comparison launch description."""
    rviz_config = os.path.join(
        get_package_share_directory('pobs_gazebo'),
        'rviz', 'trajectory_comparison.rviz')
    # Gazebo and the sensor bridge must be started separately.
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the lake trajectory comparison view'),
        DeclareLaunchArgument(
            'estimator', default_value='true',
            description='Start the existing state estimator with simulation time'),
        DeclareLaunchArgument(
            'publish_odom_transform', default_value='true',
            description='Publish lake -> odom; disable if another node owns it'),
        DeclareLaunchArgument('odom_offset_x', default_value='-12.0'),
        DeclareLaunchArgument('odom_offset_y', default_value='-8.0'),
        DeclareLaunchArgument(
            'reset_settle_s', default_value='0.25',
            description='Skip initial simulation seconds after a clock reset'),
        Node(
            package='pobs_state_estimator',
            executable='state_estimator',
            name='state_estimator',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('estimator')),
            output='screen',
        ),
        Node(
            package='pobs_gazebo',
            executable='trajectory_comparison',
            name='trajectory_comparison',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_odom_transform': ParameterValue(
                    LaunchConfiguration('publish_odom_transform'), value_type=bool),
                'odom_offset_x': ParameterValue(
                    LaunchConfiguration('odom_offset_x'), value_type=float),
                'odom_offset_y': ParameterValue(
                    LaunchConfiguration('odom_offset_y'), value_type=float),
                'reset_settle_s': ParameterValue(
                    LaunchConfiguration('reset_settle_s'), value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='trajectory_rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47
    ])
