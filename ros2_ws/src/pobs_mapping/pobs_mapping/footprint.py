"""Project sampled sonar footprints into a sparse world-frame voxel map."""

import math

import numpy as np

from pobs_mapping.geometry import quaternion_matrix


BEAM_HALF_ANGLE = math.radians(8)
BEAM_SLOPE = math.tan(BEAM_HALF_ANGLE)


def beam_footprint(hits, transform, position, quaternion):
    """Return the circle center, radius, and highest sampled world elevation."""
    hits = hits[np.all(np.isfinite(hits), axis=1)]
    forward = hits[:, 0]
    # The lidar samples a rectangular angular grid. Keep its circular cone.
    in_beam = ((forward > 0)
               & (np.hypot(hits[:, 1], hits[:, 2]) <= forward * (BEAM_SLOPE + 1e-6)))
    hits = hits[in_beam]
    if not len(hits):
        return None

    body_to_world = quaternion_matrix(quaternion)
    origin = position + body_to_world @ transform[:3, 3]
    sensor_to_world = body_to_world @ transform[:3, :3]
    elevations = origin[2] + hits @ sensor_to_world[2]
    distances = np.linalg.norm(hits, axis=1)
    height = float(np.max(elevations))
    # Equally high hits give the same elevation. Use the nearest one's range
    # so the footprint does not depend on the simulator's ray ordering.
    distance = float(np.min(distances[elevations == height]))
    center = origin + distance * sensor_to_world[:, 0]
    return center[0], center[1], distance * BEAM_SLOPE, height


class FootprintMap:
    """Retain occupied 3D cells and the latest timestamp at each location."""

    def __init__(self, transforms, cell_size):
        if not math.isfinite(cell_size) or cell_size <= 0:
            raise ValueError('footprint_cell_size_m must be finite and positive')
        self.transforms = transforms
        self.cell_size = cell_size
        self.cells = {}

    def clear(self):
        self.cells.clear()

    def add(self, sensor, hits, position, quaternion, stamp):
        footprint = beam_footprint(hits, self.transforms[sensor], position, quaternion)
        if footprint is None:
            return False
        transform = self.transforms[sensor]
        rotation = quaternion_matrix(quaternion) @ transform[:3, :3]
        center = np.array([footprint[0], footprint[1], footprint[3]])
        radius = footprint[2]
        # Sample the disk perpendicular to the sonar axis into world voxels.
        # Include its center even when the footprint is smaller than a voxel.
        count = max(1, math.ceil(radius / (self.cell_size / 2)))
        offsets = np.linspace(-radius, radius, 2 * count + 1)
        changed = False
        for u in offsets:
            for v in offsets:
                if u*u + v*v > radius*radius:
                    continue
                point = center + u * rotation[:, 1] + v * rotation[:, 2]
                key = tuple(np.floor(point / self.cell_size).astype(int))
                previous = self.cells.get(key)
                height = float(point[2])
                if previous is None or (stamp, -height) > (previous[0], -previous[1]):
                    self.cells[key] = (stamp, height)
                    changed |= previous is None
        return changed

    def paint(self, x, y, radius, elevation, stamp):
        """Update covered cells and report whether the visible surface changed."""
        size = self.cell_size
        min_i = math.ceil((x - radius) / size - 0.5)
        max_i = math.floor((x + radius) / size - 0.5)
        min_j = math.ceil((y - radius) / size - 0.5)
        max_j = math.floor((y + radius) / size - 0.5)
        radius_squared = radius * radius
        changed = False
        for i in range(min_i, max_i + 1):
            dx_squared = ((i + 0.5) * size - x) ** 2
            for j in range(min_j, max_j + 1):
                if dx_squared + ((j + 0.5) * size - y) ** 2 > radius_squared:
                    continue
                key = (i, j)
                previous = self.cells.get(key)
                if previous is None or (stamp, -elevation) > (previous[0], -previous[1]):
                    self.cells[key] = (stamp, elevation)
                    changed |= previous is None or previous[1] != elevation
        return changed

    def elevations(self):
        """Yield occupied voxel centers, including legacy elevation cells."""
        for key in self.cells:
            if len(key) == 2:
                i, j = key
                yield (i + 0.5) * self.cell_size, (j + 0.5) * self.cell_size, self.cells[key][1]
                continue
            yield tuple((index + 0.5) * self.cell_size for index in key)
