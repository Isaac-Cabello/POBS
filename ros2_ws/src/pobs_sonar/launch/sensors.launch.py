"""Start the simulation depth/IMU pipeline and all four sonar instances."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess,
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Own sensor processes together; Gazebo is started separately."""
    package_dir = get_package_share_directory('pobs_sonar')
    workspace = os.environ.get('POBS_WORKSPACE', os.path.expanduser('~/ros2_ws'))
    bridge = ExecuteProcess(
        cmd=['bash', LaunchConfiguration('bridge_script')],
        name='existing_pobs_sensors',
        condition=IfCondition(LaunchConfiguration('existing_sensors')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='hardware',
            description='Mapping sonar source: hardware or simulation'),
        DeclareLaunchArgument(
            'config', default_value=os.path.join(package_dir, 'config', 'sonars.yaml'),
            description='UART settings for left, right, middle, and downward sonars'),
        DeclareLaunchArgument(
            'existing_sensors', default_value='true',
            description='Start the existing Gazebo bridges and depth/pressure adapter'),
        DeclareLaunchArgument(
            'bridge_script', default_value=os.path.join(
                workspace, 'scripts', 'bridge_pobs.sh'),
            description='Existing sensor launcher in the source workspace'),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(
                reason='Existing sensor pipeline exited'))],
        )),
        bridge,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'sonars.launch.py')),
            launch_arguments={
                'config': LaunchConfiguration('config'),
                'source': LaunchConfiguration('source'),
            }.items(),
        ),
    ])
