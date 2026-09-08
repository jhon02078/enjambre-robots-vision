import unittest

from validar_intrinseca import build_record


def report(raw_xy, corrected_xy, detection=100.0):
    return {
        "raw_robot_poses_m": {
            "2": {"x_m": raw_xy[0], "y_m": raw_xy[1], "yaw_rad": 0.0, "yaw_deg": 0.0}
        },
        "corrected_robot_poses_m": {
            "2": {
                "x_m": corrected_xy[0], "y_m": corrected_xy[1],
                "yaw_rad": 0.0, "yaw_deg": 0.0,
            }
        },
        "detection_rate_pct": {"2": detection},
        "fallback_camera_pose_m": [0.6, 0.6, 1.8],
    }


class IntrinsicValidationTests(unittest.TestCase):
    def test_build_record_calculates_off_on_errors(self):
        record = build_record(
            "P1",
            2,
            0.5,
            0.5,
            report((0.6, 0.5), (0.55, 0.5)),
            report((0.52, 0.5), (0.51, 0.5)),
        )
        self.assertAlmostEqual(record["off_raw_error_m"], 0.1)
        self.assertAlmostEqual(record["on_raw_error_m"], 0.02)
        self.assertAlmostEqual(record["off_parallax_error_m"], 0.05)
        self.assertAlmostEqual(record["on_parallax_error_m"], 0.01)


if __name__ == "__main__":
    unittest.main()
