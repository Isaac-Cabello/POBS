"""Provisional sonar face poses in base_link (meters and radians).

Robot axes: +X forward, +Y left, +Z up. Sensor +X is the measuring axis.
Positive pitch rotates the measuring axis downward. Keep the SDF poses in sync.
"""

import math


SONAR_MOUNTS = {
    'left': (0.43, 0.15, -0.02, 0.0, math.pi / 4, 0.0),
    'right': (0.43, -0.15, -0.02, 0.0, math.pi / 4, 0.0),
    'middle': (0.44, 0.0, 0.0, 0.0, 0.0, 0.0),
    'depth': (0.0, 0.0, -0.08, 0.0, math.pi / 2, 0.0),
}
