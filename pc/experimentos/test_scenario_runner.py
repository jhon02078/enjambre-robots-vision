import json
import math
import tempfile
import time
import unittest
from pathlib import Path

from scenario_runner import (
    ScenarioRunAutomation,
    load_scenario_specs,
    minimum_pair_distance,
    order_positioning_specs,
    point_to_segment_distance,
    select_scenario_direction,
)


class ScenarioRunnerTests(unittest.TestCase):
    def test_point_to_segment_distance_handles_projection(self):
        self.assertAlmostEqual(
            point_to_segment_distance((0.5, 0.2), (0.0, 0.0), (1.0, 0.0)),
            0.2,
        )

    def test_positioning_moves_clearer_robot_first(self):
        specifications = [
            {"robot_id": 2, "start": (0.6, 1.0), "goal": (0.6, 0.2)},
            {"robot_id": 3, "start": (1.0, 0.6), "goal": (0.2, 0.6)},
        ]
        states = {
            2: {"x": 0.575, "y": 0.515},
            3: {"x": 0.648, "y": 0.741},
        }

        ordered = order_positioning_specs(specifications, states)

        self.assertEqual([item["robot_id"] for item in ordered], [3, 2])

    def test_loads_robot_starts_and_goals(self):
        payload = {
            "scenarios": {
                "cross": {
                    "robots": {
                        "2": {"start": [0.2, 0.6], "goal": [1.0, 0.6]},
                        "3": {"start": [1.0, 0.6], "goal": [0.2, 0.6]},
                    }
                }
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenarios.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            _, robots = load_scenario_specs(path, "cross")
        self.assertEqual([item["robot_id"] for item in robots], [2, 3])
        self.assertEqual(robots[0]["start"], (0.2, 0.6))

    def test_minimum_pair_distance(self):
        states = {
            2: {"x": 0.0, "y": 0.0},
            3: {"x": 0.3, "y": 0.4},
            10: {"x": 2.0, "y": 2.0},
        }
        self.assertAlmostEqual(minimum_pair_distance(states, [2, 3]), 0.5)
        self.assertTrue(math.isinf(minimum_pair_distance(states, [2])))

    def test_pose_dropout_uses_shorter_timeout_when_robots_are_close(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        runner.pose_abort_timeout_s = 1.2
        runner.close_pose_timeout_s = 0.45
        runner.close_pose_distance_m = 0.35
        runner.safety_distance_m = 0.12
        now = time.time()
        close_states = {
            2: {"x": 0.0, "y": 0.0, "t": now - 0.6},
            3: {"x": 0.2, "y": 0.0, "t": now},
        }
        far_states = {
            2: {"x": 0.0, "y": 0.0, "t": now - 0.6},
            3: {"x": 0.8, "y": 0.0, "t": now},
        }
        self.assertIn("pose_lost_close_R2", runner._safety_reason(close_states))
        self.assertIsNone(runner._safety_reason(far_states))

    def test_positioning_can_ignore_nominal_distance_but_keeps_pose_checks(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        runner.pose_abort_timeout_s = 1.2
        runner.close_pose_timeout_s = 0.45
        runner.close_pose_distance_m = 0.35
        runner.safety_distance_m = 0.12
        now = time.time()
        states = {
            2: {"x": 0.0, "y": 0.0, "t": now},
            3: {"x": 0.11, "y": 0.0, "t": now},
        }

        self.assertIn("critical_separation", runner._safety_reason(states))
        self.assertIsNone(runner._safety_reason(states, check_distance=False))

    def test_progress_reference_is_updated_only_for_robot_that_moved(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        runner.last_progress_t = time.monotonic() - 2.0
        runner.progress_reference = {
            2: (0.0, 0.0, 0.0),
            3: (0.5, 0.5, 0.0),
        }
        states = {
            2: {"x": 0.02, "y": 0.0, "yaw": 0.0},
            3: {"x": 0.505, "y": 0.5, "yaw": 0.0},
        }

        runner._update_progress(states, {2: (1.0, 0.0), 3: (0.0, 0.5)})

        self.assertEqual(runner.progress_reference[2], (0.02, 0.0, 0.0))
        self.assertEqual(runner.progress_reference[3], (0.5, 0.5, 0.0))

    def test_progress_ignores_temporarily_missing_pose(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        previous_time = time.monotonic() - 2.0
        runner.last_progress_t = previous_time
        runner.progress_reference = {3: (0.5, 0.5, 0.0)}

        runner._update_progress(
            {2: None, 3: {"x": 0.5, "y": 0.5, "yaw": 0.0}},
            {2: (1.0, 0.0), 3: (0.0, 0.5)},
        )

        self.assertEqual(runner.last_progress_t, previous_time)

    def test_stall_watchdog_tracks_each_robot_independently(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        runner.phase_started = 10.0
        runner.stall_timeout_s = 6.0
        runner.last_progress_by_robot = {2: 19.0, 3: 12.0}

        reason = runner._stalled_robot_reason(
            {2: (1.0, 0.6), 3: (0.2, 0.6)},
            now=20.0,
        )

        self.assertEqual(reason, "scenario_stalled_R3")

    def test_selects_reverse_direction_when_robots_finished_forward_run(self):
        specifications = [
            {"robot_id": 2, "start": (0.2, 0.6), "goal": (1.0, 0.6)},
            {"robot_id": 3, "start": (1.0, 0.6), "goal": (0.2, 0.6)},
        ]
        states = {
            2: {"x": 1.0, "y": 0.6},
            3: {"x": 0.2, "y": 0.6},
        }

        direction, selected, forward_cost, reverse_cost = select_scenario_direction(
            specifications, states
        )

        self.assertEqual(direction, "reverse")
        self.assertEqual(selected[0]["start"], (1.0, 0.6))
        self.assertEqual(selected[0]["goal"], (0.2, 0.6))
        self.assertGreater(forward_cost, reverse_cost)

    def test_keeps_forward_direction_when_robots_are_at_configured_starts(self):
        specifications = [
            {"robot_id": 2, "start": (0.2, 0.6), "goal": (1.0, 0.6)},
            {"robot_id": 3, "start": (1.0, 0.6), "goal": (0.2, 0.6)},
        ]
        states = {
            2: {"x": 0.2, "y": 0.6},
            3: {"x": 1.0, "y": 0.6},
        }

        direction, selected, forward_cost, reverse_cost = select_scenario_direction(
            specifications, states
        )

        self.assertEqual(direction, "forward")
        self.assertEqual(selected[0]["goal"], (1.0, 0.6))
        self.assertLess(forward_cost, reverse_cost)

    def test_network_safety_reports_robot_with_old_discovery(self):
        runner = ScenarioRunAutomation.__new__(ScenarioRunAutomation)
        runner.robot_ids = [2, 3]
        runner.network_abort_timeout_s = 5.0

        class App:
            @staticmethod
            def robot_net_status(robot_id):
                age = 6.2 if robot_id == 2 else 0.2
                return {"ip": "192.0.2.1"}, "STALE", age

        runner.app = App()
        self.assertEqual(runner._network_safety_reason(), "network_lost_R2_6.20s")

    def test_rejects_unknown_requested_direction(self):
        with self.assertRaisesRegex(ValueError, "Sentido no valido"):
            ScenarioRunAutomation(
                root=None,
                app=None,
                scenario_file="unused.json",
                scenario_name="unused",
                controller_mode="apf_fsm",
                condition="test",
                replicate=1,
                on_done=lambda *_: None,
                scenario_direction="sideways",
            )


if __name__ == "__main__":
    unittest.main()
