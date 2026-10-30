"""Publish the lake's reference geometry and register its fixed frame in TF."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from pobs_visualization.configuration import (
    DEPTH_DEEP_RGB, DEPTH_MAX_Z, DEPTH_MIN_Z, DEPTH_SHALLOW_RGB,
)


def pose_matrix(element):
    """Read the unscoped Euler poses used by the lake world and terrain model."""
    transform = np.eye(4)
    if element is None:
        return transform
    if element.get('relative_to'):
        raise ValueError('Reference lake geometry requires unscoped SDF poses')
    if element.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('Reference lake geometry requires Euler SDF poses')
    values = np.array([float(value) for value in (element.text or '').split()])
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError('Expected six finite values in lake geometry pose')
    angles = values[3:]
    if element.get('degrees', 'false').lower() in ('true', '1'):
        angles = np.deg2rad(angles)
    roll, pitch, yaw = angles
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = values[:3]
    return transform


def obj_triangles(path):
    """Read OBJ faces, including slash-separated and negative vertex indices."""
    vertices = []
    triangles = []
    for line in Path(path).read_text().splitlines():
        fields = line.split('#', 1)[0].split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == 'f':
            indices = [int(value.split('/')[0]) for value in fields[1:]]
            if any(index == 0 or not -len(vertices) <= index <= len(vertices)
                   for index in indices):
                raise ValueError(f'Invalid OBJ vertex index in {path}')
            face = [vertices[index - 1 if index > 0 else index] for index in indices]
            for index in range(1, len(face) - 1):
                triangles.extend((face[0], face[index], face[index + 1]))
    points = np.asarray(triangles, dtype=float).reshape(-1, 3)
    if not len(points) or not np.all(np.isfinite(points)):
        raise ValueError(f'No finite triangles in {path}')
    return points


def geometry_triangles(geometry, models):
    """Read the OBJ and box geometry used by the terrain and sonar test objects."""
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = mesh.findtext('uri', '')
        if not uri.startswith('model://'):
            raise ValueError(f'Unsupported lake mesh URI: {uri}')
        path = (models / uri.removeprefix('model://')).resolve()
        if not path.is_relative_to(models.resolve()):
            raise ValueError('Lake mesh is outside the models directory')
        scale = np.array([float(value) for value in
                          mesh.findtext('scale', '1 1 1').split()])
        if scale.shape != (3,) or not np.all(np.isfinite(scale)) or np.any(scale <= 0):
            raise ValueError('Mesh scale requires three finite positive values')
        return obj_triangles(path) * scale
    size = geometry.findtext('box/size')
    if size is not None:
        size = np.array([float(value) for value in size.split()])
        if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
            raise ValueError('Box size requires three finite positive values')
        vertices = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
        ]) * size / 2
        faces = np.array([
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
        ])
        return vertices[faces].reshape(-1, 3)
    raise ValueError('Reference scene supports OBJ meshes and boxes')


def lake_geometry(world_path):
    """Load the same terrain and sonar target visuals that Gazebo reads."""
    world_path = Path(world_path)
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError(f'No world in {world_path}')
    include = next((item for item in world.findall('include')
                    if item.findtext('uri') == 'model://lake_terrain'), None)
    if include is None:
        raise ValueError('World does not include model://lake_terrain')
    models = world_path.parent.parent / 'models'
    result = {}
    for include in world.findall('include'):
        uri = include.findtext('uri')
        if uri not in ('model://lake_terrain', 'model://sonar_targets'):
            continue
        folder = uri.removeprefix('model://')
        model = ET.parse(models / folder / 'model.sdf').getroot().find('model')
        world_transform = pose_matrix(include.find('pose')) @ pose_matrix(model.find('pose'))
        for link in model.findall('link'):
            link_transform = world_transform @ pose_matrix(link.find('pose'))
            for visual in link.findall('visual'):
                points = geometry_triangles(visual.find('geometry'), models)
                transform = link_transform @ pose_matrix(visual.find('pose'))
                points = points @ transform[:3, :3].T + transform[:3, 3]
                name = visual.get('name') if folder == 'lake_terrain' else \
                    f'sonar_targets/{link.get("name")}/{visual.get("name")}'
                result[name] = points
    return result


def main(args=None):
    """Retain lake reference markers for RViz subscribers joining at any time."""
    from geometry_msgs.msg import Point, TransformStamped
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import ColorRGBA
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('lake_reference_scene')
    try:
        node.declare_parameter('world_path', '')
        node.declare_parameter('world_frame', 'lake')
        world_frame = node.get_parameter('world_frame').value
        geometry = lake_geometry(node.get_parameter('world_path').value)
        # A frame name in a PointCloud2/Pose header does not create a TF frame.
        # This independent scene frame registers the root even without an estimator.
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = world_frame
        transform.child_frame_id = 'pobs_reference_scene'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = []
        snapshots = []
        obstacles = []
        for name, points in geometry.items():
            water = name == 'water_surface'
            obstacle = name.startswith('sonar_targets/')
            topic = '/pobs/visualization/lake_water' if water else \
                '/pobs/visualization/lake_terrain'
            marker = Marker()
            marker.header.frame_id = world_frame
            marker.ns = name
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b = \
                (0.02, 0.28, 0.7) if water else (0.55, 0.62, 0.35)
            alpha = 0.65 if obstacle else 0.35
            marker.color.a = 0.15 if water else alpha
            marker.points = [Point(x=float(x), y=float(y), z=float(z))
                             for x, y, z in points]
            if not water:
                # Depth colors expose the bowl and shoreline without an opaque
                # water surface hiding the sonar points beneath it.
                fraction = (DEPTH_MAX_Z - points[:, 2]) / (DEPTH_MAX_Z - DEPTH_MIN_Z)
                depth = np.clip(fraction, 0.0, 1.0)
                shallow = np.asarray(DEPTH_SHALLOW_RGB) / 255.0
                deep = np.asarray(DEPTH_DEEP_RGB) / 255.0
                colors = shallow + depth[:, None] * (deep - shallow)
                marker.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=alpha)
                                 for r, g, b in colors]
            if obstacle:
                obstacles.append(marker)
                continue
            publisher = node.create_publisher(Marker, topic, qos)
            publisher.publish(marker)
            publishers.append(publisher)
            snapshots.append((publisher, marker))
        obstacle_pub = node.create_publisher(
            MarkerArray, '/pobs/visualization/lake_obstacles', qos)
        obstacle_markers = MarkerArray(markers=obstacles)
        obstacle_pub.publish(obstacle_markers)

        def refresh_scene():
            # RViz clears its scene and TF buffer when simulation time rewinds.
            # A steady clock also restores context while Gazebo is paused.
            broadcaster.sendTransform(transform)
            for publisher, marker in snapshots:
                publisher.publish(marker)
            obstacle_pub.publish(obstacle_markers)

        node.create_timer(
            2.0, refresh_scene, clock=Clock(clock_type=ClockType.STEADY_TIME))
        node.get_logger().info(
            f'Published lake reference geometry and registered {world_frame!r} in TF')
        # Static publishers have no periodic callbacks to wake an infinite wait.
        # Bound it so Python can handle launch's SIGINT promptly on shutdown.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
