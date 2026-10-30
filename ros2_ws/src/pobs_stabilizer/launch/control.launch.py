"""Connect the stabilizer, existing SDF allocator and Gazebo command bridge."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from pobs_stabilizer.thruster_allocator import discover_thrusters


def generate_launch_description():
    # Match pobs sim's default simple model and the stabilizer's mass model.
    model = Path.home() / 'ros2_ws/src/pobs_gazebo/models/pobs/model.sdf'
    thrusters = discover_thrusters(model, 'base_link', 10.0)
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + [
                t['topic'] + '@std_msgs/msg/Float64]gz.msgs.Double'
                for t in thrusters],
            output='screen',
        ),
        Node(package='pobs_stabilizer', executable='thruster_allocator',
             parameters=[{'sdf_path': str(model), 'force_cap': 10.0}],
             output='screen'),
        Node(package='pobs_stabilizer', executable='stabilizer', output='screen'),
    ])
