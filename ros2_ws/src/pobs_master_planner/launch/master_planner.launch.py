from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pobs_master_planner'), 'config', 'planner.yaml')
    return LaunchDescription([
        Node(package='pobs_master_planner', executable='master_planner',
             name='master_planner', output='screen', parameters=[config]),
    ])
