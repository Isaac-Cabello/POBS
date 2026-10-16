"""Launch trajectory comparison and optionally its configured RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    ])
