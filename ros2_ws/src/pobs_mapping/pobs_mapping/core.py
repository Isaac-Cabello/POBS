"""Timestamped pose interpolation and persistent sonar point accumulation."""

from bisect import bisect_left

import numpy as np

from pobs_mapping.geometry import normalized_quaternion, quaternion_matrix


class PoseBuffer:
    """Sorted pose history; interpolate within history without extrapolation."""

    def __init__(self, max_age_ns=10_000_000_000):
        if max_age_ns <= 0:
            raise ValueError('Pose history duration must be positive')
        self.max_age_ns = int(max_age_ns)
        self._stamps = []
        self._poses = []

    @property
    def oldest_stamp_ns(self):
        return self._stamps[0] if self._stamps else None

    @property
    def newest_stamp_ns(self):
        return self._stamps[-1] if self._stamps else None

    def clear(self):
        self._stamps.clear()
        self._poses.clear()

    def add(self, stamp_ns, position, quaternion):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('Position must contain three finite values')
        pose = (position.copy(), normalized_quaternion(quaternion))
        stamp_ns = int(stamp_ns)
        index = bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            self._poses[index] = pose
        else:
            self._stamps.insert(index, stamp_ns)
            self._poses.insert(index, pose)
        cutoff = self._stamps[-1] - self.max_age_ns
        # Preserve one sample before cutoff for interpolation at the boundary.
        remove = max(0, bisect_left(self._stamps, cutoff) - 1)
        del self._stamps[:remove]
        del self._poses[:remove]

    def sample(self, stamp_ns, max_gap_ns=None):
        index = bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            p, q = self._poses[index]
            return p.copy(), q.copy()
        if index == 0 or index == len(self._stamps):
            return None
        if (max_gap_ns is not None and
                self._stamps[index] - self._stamps[index-1] > max_gap_ns):
            return None
        fraction = ((stamp_ns - self._stamps[index-1]) /
                    (self._stamps[index] - self._stamps[index-1]))
        p0, q0 = self._poses[index-1]
        p1, q1 = self._poses[index]
        dot = float(np.dot(q0, q1))
        if dot < 0:
            q1 = -q1
            dot = -dot
        dot = np.clip(dot, -1., 1.)
        if dot > 0.9995:
            quaternion = normalized_quaternion(q0 + fraction*(q1-q0))
        else:
            angle = np.arccos(dot)
            quaternion = (np.sin((1-fraction)*angle)*q0 +
                          np.sin(fraction*angle)*q1) / np.sin(angle)
        return p0 + fraction*(p1-p0), quaternion


class PointAccumulator:
    """Retain every valid hit, using sensor +X as the beam centerline."""

    def __init__(self, transforms):
        self.transforms = {name: np.asarray(value, dtype=float).copy()
                           for name, value in transforms.items()}
        self.points = []

    def clear(self):
        self.points.clear()

    def add(self, sensor_name, distance, min_range, max_range, position, quaternion):
        values = [distance, min_range, max_range]
        if (not np.all(np.isfinite(values)) or min_range < 0 or
                max_range < min_range or not min_range <= distance <= max_range):
            return False
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return False
        transform = self.transforms[sensor_name]
        local = transform[:3, 3] + transform[:3, 0]*distance
        point = position + quaternion_matrix(quaternion) @ local
        self.points.append(tuple(float(value) for value in point))
        return True
