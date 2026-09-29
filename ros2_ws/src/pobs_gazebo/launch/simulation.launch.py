from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    pobs_gazebo_share = get_package_share_directory("pobs_gazebo")

    world = os.path.join(
        pobs_gazebo_share,
        "worlds",
        "empty.sdf",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    ros_gz_sim_share,
                    "launch",
                    "gz_server.launch.py",
                )
            ),
            launch_arguments={
                "world_sdf_file": world,
            }.items(),
        )
    ])
