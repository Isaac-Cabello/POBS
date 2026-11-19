"""Simulation-only planner test; no thrust commands, no physical hardware."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('pobs_master_planner'),
        'config', 'planner.yaml'
    )
    return LaunchDescription([
        Node(
            package='pobs_master_planner',
            executable='gazebo_pose_to_odom',
            name='gazebo_pose_to_odom',
            output='screen',
        ),
        Node(
            package='pobs_master_planner',
            executable='master_planner',
            name='master_planner',
            output='screen',
            parameters=[cfg, {'require_calibration': False}],
            remappings=[
                ('/state/odometry', '/planner/sim_odometry'),
                ('/controller/target_pose', '/planner/dry_run_target_pose'),
            ],
        ),
    ])
