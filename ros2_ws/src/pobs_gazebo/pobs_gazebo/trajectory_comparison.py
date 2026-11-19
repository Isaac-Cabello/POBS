"""Compatibility entry point for the original Gazebo comparison node."""

from pobs_visualization.trajectory_comparison import TrajectoryComparison, main

__all__ = ['TrajectoryComparison', 'main']

if __name__ == '__main__':
    main()
