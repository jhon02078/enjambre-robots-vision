import tempfile
import time
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from analizar_experimento import summarize_run
from runtime import DelayedCommandDispatcher, ExperimentRecorder


class ExperimentPipelineTest(unittest.TestCase):
    def test_recorder_and_analysis(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            recorder = ExperimentRecorder(root / "raw", repo_root=root)
            run_dir = recorder.start({
                "scenario": "synthetic_crossing",
                "condition": "apf_fsm",
                "replicate": 1,
                "controller_mode": "apf_fsm",
                "requested_delay_ms": 50.0,
                "collision_threshold_m": 0.12,
            })
            for frame_seq in range(30):
                progress = frame_seq / 29.0
                positions = {
                    1: (0.15 + 0.80 * progress, 0.50),
                    2: (0.95 - 0.80 * progress, 0.72),
                }
                for rid, (x, y) in positions.items():
                    recorder.frame(
                        frame_seq=frame_seq,
                        robot_id=rid,
                        detected=1,
                        accepted=1,
                        homography_valid=1,
                        x=x,
                        y=y,
                        yaw=0.0,
                        floor_x=x + 0.01,
                        floor_y=y,
                        corrected_x=x,
                        corrected_y=y,
                        raw_yaw=0.0,
                    )
                    recorder.control(
                        cycle_id=frame_seq,
                        robot_id=rid,
                        state="RUN",
                        previous_state="RUN",
                        x=x,
                        y=y,
                        yaw=0.0,
                        waypoint_x=1.0 if rid == 1 else 0.0,
                        waypoint_y=y,
                        final_goal_x=1.0 if rid == 1 else 0.0,
                        final_goal_y=y,
                        distance_error=abs((1.0 if rid == 1 else 0.0) - x),
                        desired_heading=0.0,
                        angle_error=0.02,
                        u_att_x=1.0,
                        u_att_y=0.0,
                        u_rep_x=0.0,
                        u_rep_y=0.0,
                        u_res_x=1.0,
                        u_res_y=0.0,
                        repulsion_norm=0.0,
                        align_factor=1.0,
                        linear_cmd=30.0,
                        angular_cmd=1.0,
                        left_cmd=29,
                        right_cmd=31,
                        controller_mode="apf_fsm",
                    )
                recorder.network(stage="sent", robot_id=1, seq=frame_seq, success=1)
                recorder.network(stage="ack", robot_id=1, seq=frame_seq, rtt_ms=8.0, success=1)
                time.sleep(0.001)
            recorder.event("target_reached", robot_id=1)
            recorder.event("target_reached", robot_id=2)
            recorder.stop("synthetic_complete")

            summary = summarize_run(run_dir)
            self.assertEqual(len(summary["robots"]), 2)
            self.assertEqual(summary["acks_received"], 30)
            self.assertAlmostEqual(summary["rtt_mean_ms"], 8.0, places=5)
            self.assertTrue((run_dir / "processed" / "trajectories.png").exists())
            self.assertTrue((run_dir / "processed" / "timeseries.png").exists())
            self.assertTrue((run_dir / "processed" / "summary.json").exists())

    def test_delayed_dispatcher(self):
        received = []
        dispatcher = DelayedCommandDispatcher(
            lambda item, delay_ms: received.append((item["value"], delay_ms))
        )
        started = time.perf_counter()
        dispatcher.schedule(
            started + 0.03,
            {"value": 7, "robot_id": 1, "scheduled_perf": started},
        )
        time.sleep(0.08)
        dispatcher.close()
        self.assertEqual(received[0][0], 7)
        self.assertGreaterEqual(received[0][1], 20.0)


if __name__ == "__main__":
    unittest.main()

