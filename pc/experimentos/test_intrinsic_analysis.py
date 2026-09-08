import unittest

from analizar_intrinseca import paired_metrics, summarize


class IntrinsicAnalysisTests(unittest.TestCase):
    def test_paired_metrics_contains_mean_in_confidence_interval(self):
        result = paired_metrics([-1.0, 0.0, 1.0, 2.0])
        self.assertLess(result["ci95_low"], result["mean"])
        self.assertGreater(result["ci95_high"], result["mean"])

    def test_summary_reports_paired_improvement(self):
        rows = [
            {
                "off_raw_error_m": 0.10, "on_raw_error_m": 0.08,
                "off_parallax_error_m": 0.05, "on_parallax_error_m": 0.03,
                "off_detection_rate_pct": 80.0, "on_detection_rate_pct": 100.0,
            },
            {
                "off_raw_error_m": 0.12, "on_raw_error_m": 0.09,
                "off_parallax_error_m": 0.04, "on_parallax_error_m": 0.04,
                "off_detection_rate_pct": 90.0, "on_detection_rate_pct": 100.0,
            },
        ]
        result = summarize(rows)
        self.assertEqual(result["points"], 2)
        self.assertEqual(result["stages"]["raw"]["on_better_points"], 2)
        self.assertAlmostEqual(result["detection"]["paired_improvement_mean_points"], 15.0)


if __name__ == "__main__":
    unittest.main()
