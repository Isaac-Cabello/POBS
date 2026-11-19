import math
import unittest

def recover_velocity(previous_velocity, measured_velocity, alpha=0.25):
    """Apply the same smoothing rule used by the XY controller."""
    if all(math.isfinite(v) for v in previous_velocity):
        return (
            alpha * measured_velocity[0] + (1 - alpha) * previous_velocity[0],
            alpha * measured_velocity[1] + (1 - alpha) * previous_velocity[1],
        )
    return measured_velocity

class TestControllerRegression(unittest.TestCase):

    def test_velocity_recovers_from_nan(self):
        previous = (float('nan'), float('nan'))
        measured = (0.1, 0.2)

        result = recover_velocity(previous, measured)

        self.assertEqual(result, measured)
        self.assertTrue(all(math.isfinite(v) for v in result))

    def test_velocity_smoothing(self):
        previous = (0.0, 0.0)
        measured = (0.4, 0.8)

        result = recover_velocity(previous, measured)

        self.assertAlmostEqual(result[0], 0.1)
        self.assertAlmostEqual(result[1], 0.2)


if __name__ == '__main__':
    unittest.main()
