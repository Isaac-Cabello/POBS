"""Own isolated Gazebo bridge children and terminate them once on shutdown."""

import os
import signal
import subprocess
import sys
import threading


def run_managed(command):
    """Keep terminal and launch SIGINT forwarding away from the bridge child."""
    stop = threading.Event()
    previous = {}

    def request_stop(signum, frame):
        del signum, frame
        stop.set()

    child = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        child = subprocess.Popen(command, start_new_session=True)
        while child.poll() is None and not stop.wait(0.05):
            pass
        return 0 if stop.is_set() else child.returncode
    except OSError as error:
        print(f'Unable to start Gazebo bridge: {error}', file=sys.stderr)
        return 1
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def main():
    """Run the scan bridge with the exact topic/remap arguments from launch."""
    from ament_index_python.packages import get_package_prefix

    executable = os.path.join(
        get_package_prefix('ros_gz_bridge'), 'lib', 'ros_gz_bridge', 'parameter_bridge')
    return run_managed([executable, *sys.argv[1:]])


if __name__ == '__main__':
    sys.exit(main())
