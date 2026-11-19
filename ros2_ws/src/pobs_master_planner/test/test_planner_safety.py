import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from builtin_interfaces.msg import Time
from pobs_master_planner.master_planner import MasterPlanner
from pobs_master_planner.sim_thruster_safety import SimThrusterSafety


class TestMasterPlanner(unittest.TestCase):

    def make_planner(self):
        self.clock_time = 100.0

        params = {
            'odom_timeout_s': 1.0,
            'require_calibration': False,
            'goal_tolerance_m': 0.20,
            'stop_speed_mps': 0.03,
            'settle_time_s': 1.0,
            'fixed_z_m': -3.0,
        }

        return SimpleNamespace(
            now_s=lambda: self.clock_time,
            get_parameter=lambda name: SimpleNamespace(value=params[name]),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: Time())
            ),
            goal=(0.5, 0.5),
            odom=(0.5, 0.5),
            last_odom_time=100.0,
            velocity_xy=(0.0, 0.0),
            arrival_since=None,
            state='NAVIGATING',
            calibrated=False,
            fault=None,
            target_pub=Mock(),
            status_pub=Mock(),
            mode_pub=Mock(),
            publish_reached=Mock(),
        )

    def test_goal_requires_settle_time(self):
        planner = self.make_planner()

        MasterPlanner.step(planner)
        self.assertEqual(planner.state, 'NAVIGATING')

        self.clock_time = 100.5
        planner.last_odom_time = self.clock_time
        MasterPlanner.step(planner)
        self.assertEqual(planner.state, 'NAVIGATING')

        self.clock_time = 101.1
        planner.last_odom_time = self.clock_time
        MasterPlanner.step(planner)

        self.assertEqual(planner.state, 'GOAL_REACHED')
        planner.publish_reached.assert_called_with(True)

    def test_overshoot_recovery(self):
            planner = self.make_planner()

            MasterPlanner.step(planner)

            self.clock_time = 101.1
            planner.last_odom_time = self.clock_time
            MasterPlanner.step(planner)

            self.assertEqual(planner.state, 'GOAL_REACHED')

            # Simulate drifting outside the 20 cm tolerance.
            planner.odom = (0.8, 0.5)
            self.clock_time = 101.2
            planner.last_odom_time = self.clock_time

            MasterPlanner.step(planner)

            self.assertEqual(planner.state, 'NAVIGATING')
            self.assertEqual(planner.goal, (0.5, 0.5))
            self.assertIsNone(planner.arrival_since)
            planner.publish_reached.assert_called_with(False)

class TestSimulationSafety(unittest.TestCase):

    def make_gate(self, xy):
        params = {
            'enabled': True,
            'max_force_n': 0.5,
        }

        return SimpleNamespace(
            get_parameter=lambda name: SimpleNamespace(value=params[name]),
            xy=xy,
            pitch=0.0,
            roll=0.0,
            pose_time=99.9,
            planner_time=99.9,
            controller_time=99.9,
            planner_state='NAVIGATING',
            controller_state='TRACKING',
            cmd={4: 0.3, 5: -0.3},
            cmd_time={4: 99.9, 5: 99.9},
            pub4=Mock(),
            pub5=Mock(),
            status_pub=Mock(),
        )

    def get_result(self, gate):
        message = gate.status_pub.publish.call_args.args[0]
        return json.loads(message.data)

    def test_outside_boundary_blocks_thrust(self):
        gate = self.make_gate((3.1, 0.0))

        with patch(
            'pobs_master_planner.sim_thruster_safety.time.monotonic',
            return_value=100.0
        ):
            SimThrusterSafety.update(gate)

        result = self.get_result(gate)

        self.assertEqual(result['state'], 'OUTSIDE_TEST_AREA')
        self.assertEqual(result['force_4_n'], 0.0)
        self.assertEqual(result['force_5_n'], 0.0)

    def test_inside_boundary_allows_thrust(self):
        gate = self.make_gate((2.9, 0.0))

        with patch(
            'pobs_master_planner.sim_thruster_safety.time.monotonic',
            return_value=100.0
        ):
            SimThrusterSafety.update(gate)

        result = self.get_result(gate)

        self.assertEqual(result['state'], 'ACTIVE')
        self.assertAlmostEqual(result['force_4_n'], 0.15)
        self.assertAlmostEqual(result['force_5_n'], -0.15)


if __name__ == '__main__':
    unittest.main()
