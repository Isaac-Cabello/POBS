"""Fixed world-elevation colors for sonar and reference terrain."""

DEPTH_MIN_Z = -10.0
DEPTH_MAX_Z = 0.0
DEPTH_DEEP_RGB = (16, 30, 85)
DEPTH_MID_RGB = (40, 90, 194)
DEPTH_SHALLOW_RGB = (168, 206, 255)


def elevation_rgb(z):
    """Return normalized RGB, with deeper elevations shown in darker blue."""
    fraction = min(1.0, max(0.0, (z - DEPTH_MIN_Z) / (DEPTH_MAX_Z - DEPTH_MIN_Z)))
    if fraction <= 0.5:
        start, end = DEPTH_DEEP_RGB, DEPTH_MID_RGB
        blend = fraction * 2
    else:
        start, end = DEPTH_MID_RGB, DEPTH_SHALLOW_RGB
        blend = (fraction - 0.5) * 2
    return tuple((a + blend * (b - a)) / 255 for a, b in zip(start, end))
