"""Start the mounted sonar frames and select hardware or simulation ranges."""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from pobs_sonar.mounting import SONAR_MOUNTS


def sensor_actions(context):
    """Select one data source and publish sensor mounting frames."""
    source = LaunchConfiguration('source').perform(context)
    if source not in ('hardware', 'simulation'):
        raise ValueError('source must be hardware or simulation')
    actions = []
    for position, pose in SONAR_MOUNTS.items():
        x, y, z, roll, pitch, yaw = pose
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'sonar_{position}_mount',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                '--frame-id', 'base_link',
                '--child-frame-id', f'sonar_{position}_link',
            ], output='screen',
        ))
    if source == 'hardware':
        actions.extend([
            Node(
                package='pobs_sonar', executable='sen0599_driver',
                namespace='/pobs/sonar', name=f'sonar_{position}',
                parameters=[LaunchConfiguration('config')], output='screen',
            ) for position in SONAR_MOUNTS
        ])
    else:
        actions.extend([
            ExecuteProcess(
                name='mapping_sonar_bridge',
                cmd=[sys.executable, '-m', 'pobs_sonar.managed_bridge', *[
                    f'/pobs/sonar/sonar_{position}/scan'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                    for position in SONAR_MOUNTS
                ], '--ros-args', '-r', '__node:=mapping_sonar_bridge'], output='screen',
            ),
            Node(
                package='pobs_sonar', executable='simulated_sonars',
                name='simulated_sonars', output='screen',
            ),
        ])
    return actions


def generate_launch_description():
    """Use a shared config file to name and configure all four sensors."""
    config = os.path.join(
        get_package_share_directory('pobs_sonar'), 'config', 'sonars.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='hardware',
            description='Range source: hardware UART sensors or mounted Gazebo sensors'),
        DeclareLaunchArgument(
            'config', default_value=config,
            description='YAML parameters for all four SEN0599 nodes'),
        OpaqueFunction(function=sensor_actions),
    ])
