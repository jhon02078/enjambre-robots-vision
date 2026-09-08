import sys
import unittest
from pathlib import Path

import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parent
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from snapshot_controller import MultiRobotApp, fsm_tangential_component


class ApfHelperTests(unittest.TestCase):
    def test_head_on_robots_receive_opposite_lateral_directions(self):
        left_robot = fsm_tangential_component(-0.2, 0.0, 0.2, 0.28)
        right_robot = fsm_tangential_component(0.2, 0.0, 0.2, 0.28)
        self.assertLess(left_robot[1], 0.0)
        self.assertGreater(right_robot[1], 0.0)
        np.testing.assert_allclose(left_robot, -right_robot)

    def test_component_is_tangent_to_radial_vector(self):
        tangent = fsm_tangential_component(0.12, 0.16, 0.2, 0.28)
        self.assertAlmostEqual(float(np.dot(tangent, [0.12, 0.16])), 0.0, places=6)

    def test_component_is_zero_outside_influence_radius(self):
        tangent = fsm_tangential_component(0.3, 0.0, 0.3, 0.28)
        np.testing.assert_array_equal(tangent, np.zeros(2, dtype=np.float32))

    def test_navigation_recovery_starts_after_pose_stalls(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.nav_last_pose = {2: None}
        app.nav_last_motion_t = {2: None}
        app.nav_recovery_until = {2: 0.0}
        app.nav_recovery_next_t = {2: 0.0}

        self.assertEqual(
            app._navigation_recovery_status(2, 0.5, 0.5, 0.0, now=10.0),
            (False, False),
        )
        self.assertEqual(
            app._navigation_recovery_status(2, 0.5, 0.5, 0.0, now=10.61),
            (True, True),
        )
        self.assertEqual(
            app._navigation_recovery_status(2, 0.5, 0.5, 0.0, now=10.70),
            (True, False),
        )

    def test_navigation_progress_cancels_next_recovery(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.nav_last_pose = {2: (0.5, 0.5, 0.0)}
        app.nav_last_motion_t = {2: 10.0}
        app.nav_recovery_until = {2: 10.5}
        app.nav_recovery_next_t = {2: 10.75}

        self.assertEqual(
            app._navigation_recovery_status(2, 0.51, 0.5, 0.0, now=10.8),
            (False, False),
        )
        self.assertEqual(app.nav_last_motion_t[2], 10.8)


if __name__ == "__main__":
    unittest.main()
