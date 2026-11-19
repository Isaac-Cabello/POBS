import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('pobs_state_estimator'),
        'config',
        'bno055_pobs.yaml',
    )
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[config_file],
    )
    state_estimator_node = Node(
        package='pobs_state_estimator',
        executable='state_estimator',
        name='state_estimator',
        output='screen',
        parameters=[{
            'imu_topic': '/pobs/imu',
            'depth_topic': '/pobs/depth',
            'imu_acceleration_includes_gravity': False,
            'use_sim_time': False,
            'publish_rate_hz': 100.0,
        }],
    )
    return LaunchDescription([
        bno055_node,
        state_estimator_node,
    ])
