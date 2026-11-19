# NEVER USE FOR PHYSICAL HARDWARE
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pobs_master_planner'),
        'config',
        'planner.yaml',
    )
    sim_odom = [
        ('/state/odometry','/planner/sim_odometry'),
    ]
    return LaunchDescription([
        Node(
            package='pobs_master_planner',
            executable='gazebo_pose_to_odom',
            name='gaebo_pose_to_odom',
            output='screen',
            parameters=[{
                'pose_topic':'/model/pobs_balance_test/pose',
                'odom_topic':'/planner/sim_odometry',
            }]
        ),
        Node(
            package='pobs_master_planner',
            executable='master_planner',
            name='master_planner',
            output='screen',
            parameters=[config,{'require_calibration': False}],
            remappings=sim_odom,
        ),
        Node(
            package='pobs_master_planner',
            executable='xy_controller',
            name='xy_controller',
            output='screen',
            parameters=[{'require_level':True}],
            remappings=sim_odom,
        ),
        Node(
            package='pobs_master_planner',
            executable='sim_thruster_safety',
            name='sim_thruster_safety',
            output='screen',
            parameters=[{
                'enabled':False,
                'max_force_n':0.5,
                'gazebo_namespace': 'pobs',
            }],
        ),
    ])