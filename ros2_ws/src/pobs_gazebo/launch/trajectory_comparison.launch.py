"""Launch trajectory comparison and optionally its configured RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the comparison launch description."""
    rviz_config = os.path.join(
        get_package_share_directory('pobs_gazebo'),
        'rviz', 'trajectory_comparison.rviz')
    # Inputs must be started separately; this launch does not start Gazebo
    # or an estimator.
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the lake trajectory comparison view'),
        Node(
            package='pobs_gazebo',
            executable='trajectory_comparison',
            name='trajectory_comparison',
            output='screen',
            parameters=[{'use_sim_time': True}],
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
