#!/usr/bin/env python3
"""Move the Gazebo robot through a back-and-forth sonar survey."""

import argparse
from collections import defaultdict
import math
from pathlib import Path
import sys
import time

import numpy as np

from generate_lake_terrain import read_obj, terrain_height


LAKE_MESH = (Path(__file__).resolve().parents[1] / 'src/pobs_gazebo/models'
             / 'lake_terrain/meshes/terrain_bumpy.obj')
MIN_DEPTH_M = 0.6
MIN_SUBMERGENCE_M = 0.25
BIN_SIZE_M = 2.0


def terrain_index(vertices, faces):
    """Group the existing OBJ triangles by XY so height queries stay local."""
    triangles = vertices[faces, :2]
    lower = np.floor(triangles.min(axis=1) / BIN_SIZE_M).astype(int)
    upper = np.floor(triangles.max(axis=1) / BIN_SIZE_M).astype(int)
    bins = defaultdict(list)
    for face, ((xmin, ymin), (xmax, ymax)) in enumerate(zip(lower, upper)):
        for i in range(xmin, xmax + 1):
            for j in range(ymin, ymax + 1):
                bins[i, j].append(face)
    return {key: faces[indices] for key, indices in bins.items()}


def axis_samples(start, stop, spacing):
    """Include both endpoints and keep every sample interval within spacing."""
    return np.linspace(start, stop, math.ceil((stop - start) / spacing) + 1)


def survey_rows(mesh, bounds, lane_spacing, tick_spacing, altitude):
    """Plan the submerged part of each lane at a terrain-relative altitude."""
    vertices, faces = read_obj(mesh)
    bins = terrain_index(vertices, faces)
    if bounds is None:
        wet = vertices[vertices[:, 2] < 0, :2]
        if not len(wet):
            raise ValueError('Terrain has no vertices below the water surface')
        xmin, ymin = wet.min(axis=0)
        xmax, ymax = wet.max(axis=0)
    else:
        xmin, xmax, ymin, ymax = bounds
    xs = axis_samples(xmin, xmax, tick_spacing)
    for index, y in enumerate(axis_samples(ymin, ymax, lane_spacing)):
        reverse = index % 2 == 1
        poses = []
        for x in xs[::-1] if reverse else xs:
            local_faces = bins.get((math.floor(x / BIN_SIZE_M), math.floor(y / BIN_SIZE_M)))
            if local_faces is None:
                continue
            try:
                floor = terrain_height(vertices, local_faces, x, y)
            except ValueError:  # The bin can extend past the edge of the mesh.
                continue
            if floor <= -MIN_DEPTH_M:
                z = min(-MIN_SUBMERGENCE_M, floor + altitude)
                poses.append((float(x), float(y), float(z), reverse))
        if poses:
            yield poses


class SurveyClock:
    """Advance on Gazebo simulation time, regardless of wall-clock speed."""

    def __init__(self, spin_once):
        self.spin_once = spin_once
        self.ns = 0
        self.rewound = False

    def receive(self, message):
        current = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        self.rewound |= current < self.ns
        self.ns = current

    def wait_until(self, target_ns):
        deadline = time.monotonic() + 5
        while self.ns < target_ns and not self.rewound:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError('Gazebo clock stopped advancing. Check that the world is running.')
            self.spin_once(remaining)
        if self.rewound:
            raise RuntimeError('Gazebo restarted during the survey. Start a new survey.')
        return self.ns


def run_survey(args, rows):
    """Set each pose through Gazebo and leave the world running afterwards."""
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.pose_pb2 import Pose
    from gz.transport import Node
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock

    # Gazebo's synchronous Python service call can block Gazebo's Python
    # subscription callbacks. Read the bridged clock through ROS instead.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    ros_node = rclpy.create_node('lake_survey')
    clock = SurveyClock(lambda timeout: rclpy.spin_once(ros_node, timeout_sec=timeout))
    ros_node.create_subscription(Clock, args.clock_topic, clock.receive, qos_profile_sensor_data)
    gazebo = Node()
    tick_ns = round(1_000_000_000 / args.rate_hz)
    next_tick = 1
    service = f'/world/{args.world}/set_pose/blocking'
    try:
        for index, row in enumerate(rows, 1):
            print(f'Lane {index}/{len(rows)} at y={row[0][1]:.2f} m', flush=True)
            for x, y, z, reverse in row:
                stamp = clock.wait_until(next_tick)
                pose = Pose(name=args.model)
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                pose.orientation.w = 0.0 if reverse else 1.0
                pose.orientation.z = 1.0 if reverse else 0.0
                ok, response = gazebo.request(service, pose, Pose, Boolean, 2000)
                if not ok or not response.data:
                    raise RuntimeError(f'Gazebo could not set the pose of {args.model!r} via {service}')
                next_tick = stamp + tick_ns
        # Let the 10 Hz sonars see the last commanded position before returning.
        clock.wait_until(next_tick + 100_000_000)
    finally:
        ros_node.destroy_node()
        rclpy.try_shutdown()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='lake')
    parser.add_argument('--model', default='pobs')
    parser.add_argument('--clock-topic', default='/pobs/visualization/clock',
                        help='ROS simulation clock topic (default: the mapping view clock)')
    parser.add_argument('--terrain', type=Path, default=LAKE_MESH, help='floor OBJ')
    parser.add_argument('--bounds', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                        help='survey a rectangle within the lake (meters)')
    parser.add_argument('--altitude', type=positive, default=3.5,
                        help='target height above the floor in meters (default: 3.5)')
    parser.add_argument('--fast', action='store_true', help='use 1.25 m lanes and a speed of 8 m/s')
    parser.add_argument('--lane-spacing', type=positive,
                        help='maximum distance between lanes in meters (default: 0.65)')
    parser.add_argument('--speed', type=positive,
                        help='target speed in meters per simulation second (default: 5)')
    parser.add_argument('--rate-hz', type=positive, default=20.0,
                        help='pose updates per simulation second (default: 20)')
    parser.add_argument('--dry-run', action='store_true', help='plan and print duration only')
    args = parser.parse_args()
    if args.bounds is not None:
        xmin, xmax, ymin, ymax = args.bounds
        if not all(map(math.isfinite, args.bounds)) or not (xmin < xmax and ymin < ymax):
            parser.error('--bounds requires finite XMIN < XMAX and YMIN < YMAX')
    if args.altitude < MIN_SUBMERGENCE_M:
        parser.error(f'--altitude must be at least {MIN_SUBMERGENCE_M} m')
    if args.rate_hz > 1000:
        parser.error('--rate-hz cannot exceed 1000')
    try:
        lane_spacing = args.lane_spacing or (1.25 if args.fast else 0.65)
        speed = args.speed or (8.0 if args.fast else 5.0)
        rows = list(survey_rows(
            args.terrain, args.bounds, lane_spacing, speed / args.rate_hz, args.altitude))
        if not rows:
            raise ValueError('No submerged survey poses within those bounds')
        count = sum(map(len, rows))
        duration = count / args.rate_hz
        estimate = f'{duration:.1f} seconds' if duration < 60 else f'{duration / 60:.1f} minutes'
        print(f'Planned {len(rows)} lanes and {count:,} poses, about '
              f'{estimate} of simulation time.', flush=True)
        if not args.dry_run:
            run_survey(args, rows)
            print('Survey complete. The accumulated map is ready in RViz.')
    except KeyboardInterrupt:
        print('\nSurvey stopped.')
        return 130
    except (RuntimeError, ValueError, OSError) as error:
        print(f'Survey failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
