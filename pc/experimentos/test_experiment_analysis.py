import unittest

from analizar_experimento import classify_trial_outcome, scenario_robot_ids


class ExperimentAnalysisTests(unittest.TestCase):
    def test_controller_stall_is_a_valid_task_failure(self):
        self.assertEqual(
            classify_trial_outcome("scenario_stalled_R2", False),
            (True, "task_failure"),
        )

    def test_pose_loss_is_an_infrastructure_abort(self):
        self.assertEqual(
            classify_trial_outcome("pose_lost_close_R2_0.20m", False),
            (False, "infrastructure_abort"),
        )

    def test_completed_run_is_a_valid_success(self):
        self.assertEqual(
            classify_trial_outcome("completed", True),
            (True, "success"),
        )

    def test_scenario_robot_ids_excludes_configured_but_inactive_robots(self):
        events = [
            {
                "event": "scenario_started",
                "payload": {"robots": [2, 3], "direction": "reverse"},
            }
        ]

        self.assertEqual(scenario_robot_ids(events), [2, 3])

    def test_scenario_robot_ids_uses_latest_start(self):
        events = [
            {"event": "scenario_started", "payload": {"robots": [2]}},
            {"event": "global_stop", "payload": {}},
            {"event": "scenario_started", "payload": {"robots": [2, 3]}},
        ]

        self.assertEqual(scenario_robot_ids(events), [2, 3])


if __name__ == "__main__":
    unittest.main()
