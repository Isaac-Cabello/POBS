import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from pobs_master_planner.xy_controller import XYController


class TestXYController(unittest.TestCase):

    def make_controller(self):
        """Create controller state without starting a ROS node."""
        controller = SimpleNamespace(
            goal=(1.0, 0.0),
            pose=(0.0, 0.0, 0.0, 0.0, 0.0),
            velocity_xy=(0.0, 0.0),
            prev_xy=None,
            prev_odom_time=None,
            last_odom=100.0,
            last_status=100.0,
            planner_state='NAVIGATING',
            require_level=True,
            max_tilt_deg=15.0,
            goal_tolerance=0.20,
            stop_speed_mps=0.03,
            kp_distance=0.4,
            kp_heading=0.8,
            kd_distance=1.0,
            max_forward=0.35,
            max_turn=0.30,
            left_pub=Mock(),
            right_pub=Mock(),
            status_pub=Mock(),
        )
        return controller

    def make_odom(self, x, y):
        return SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=y),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                )
            )
        )

    def feedback(self, controller):
        message = controller.status_pub.publish.call_args.args[0]
        return json.loads(message.data)

    def test_velocity_recovers_from_nan(self):
        controller = self.make_controller()
        controller.prev_xy = (0.0, 0.0)
        controller.prev_odom_time = 99.9
        controller.velocity_xy = (float('nan'), float('nan'))

        with patch(
            'pobs_master_planner.xy_controller.time.monotonic',
            return_value=100.0
        ):
            XYController.odom_callback(
                controller, self.make_odom(0.01, 0.02)
            )

        self.assertAlmostEqual(controller.velocity_xy[0], 0.1)
        self.assertAlmostEqual(controller.velocity_xy[1], 0.2)
        self.assertTrue(
            all(math.isfinite(v) for v in controller.velocity_xy)
        )

    def test_invalid_velocity_disables_thrust(self):
        controller = self.make_controller()
        controller.velocity_xy = (float('nan'), float('nan'))

        with patch(
            'pobs_master_planner.xy_controller.time.monotonic',
            return_value=100.0
        ):
            XYController.control_loop(controller)

        result = self.feedback(controller)

        self.assertEqual(result['state'], 'INVALID_VELOCITY')
        self.assertEqual(result['thruster_4'], 0.0)
        self.assertEqual(result['thruster_5'], 0.0)

    def test_controller_turns_toward_goal(self):
        controller = self.make_controller()
        controller.goal = (0.0, 1.0)

        with patch(
            'pobs_master_planner.xy_controller.time.monotonic',
            return_value=100.0
        ):
            XYController.control_loop(controller)

        result = self.feedback(controller)

        self.assertEqual(result['state'], 'TRACKING')
        self.assertLess(result['thruster_4'], 0.0)
        self.assertGreater(result['thruster_5'], 0.0)

    def test_goal_reached_disables_thrust(self):
        controller = self.make_controller()
        controller.goal = (0.1, 0.0)
        controller.velocity_xy = (0.0, 0.0)

        with patch(
            'pobs_master_planner.xy_controller.time.monotonic',
            return_value=100.0
        ):
            XYController.control_loop(controller)

        result = self.feedback(controller)

        self.assertEqual(result['state'], 'GOAL_REACHED')
        self.assertEqual(result['thruster_4'], 0.0)
        self.assertEqual(result['thruster_5'], 0.0)


if __name__ == '__main__':
    unittest.main()
