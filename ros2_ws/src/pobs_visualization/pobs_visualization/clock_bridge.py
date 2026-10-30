"""Bridge the world clock with clean ownership of the Gazebo process."""

import os
import sys

from pobs_sonar.managed_bridge import run_managed


def main():
    """Bridge the selected Gazebo world's clock without initializing a ROS node."""
    from ament_index_python.packages import get_package_prefix

    if len(sys.argv) != 2:
        print('Usage: clock_bridge WORLD_NAME', file=sys.stderr)
        return 2
    world = sys.argv[1]
    executable = os.path.join(
        get_package_prefix('ros_gz_bridge'), 'lib', 'ros_gz_bridge', 'parameter_bridge')
    return run_managed([
        executable,
        f'/world/{world}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '--ros-args', '-r', '__node:=visualization_clock_bridge',
        '-r', f'/world/{world}/clock:=/clock',
    ])


if __name__ == '__main__':
    sys.exit(main())
