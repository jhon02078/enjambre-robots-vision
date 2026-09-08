import json
import math
import tempfile
import unittest
from pathlib import Path

from localization import apply_localization_alignment, load_localization_alignment
from analizar_localizacion_grid import stage_metrics
from calibrar_alineacion_xy import fit_translation_alignment


class LocalizationAlignmentTest(unittest.TestCase):
    def test_load_and_apply_translation(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "alignment.json"
            path.write_text(
                json.dumps({
                    "method": "translation_bias_after_parallax",
                    "enabled": True,
                    "offset_x_m": -0.012,
                    "offset_y_m": 0.004,
                }),
                encoding="utf-8",
            )
            alignment = load_localization_alignment(path)
            x_m, y_m = apply_localization_alignment(0.50, 0.60, alignment)
            self.assertAlmostEqual(x_m, 0.488)
            self.assertAlmostEqual(y_m, 0.604)

    def test_disabled_alignment_preserves_pose(self):
        alignment = {
            "enabled": True,
            "offset_x_m": 0.10,
            "offset_y_m": -0.10,
        }
        self.assertEqual(
            apply_localization_alignment(0.20, 0.30, alignment, enabled=False),
            (0.20, 0.30),
        )

    def test_stage_metrics_uses_radial_error(self):
        rows = [
            {"estimate_x": 0.03, "estimate_y": 0.04, "physical_x_m": 0.0, "physical_y_m": 0.0},
            {"estimate_x": 0.00, "estimate_y": 0.00, "physical_x_m": 0.0, "physical_y_m": 0.0},
        ]
        metrics = stage_metrics(rows, "estimate_x", "estimate_y")
        self.assertAlmostEqual(metrics["mean_error_m"], 0.025)
        self.assertAlmostEqual(metrics["max_error_m"], 0.05)
        self.assertAlmostEqual(metrics["rmse_radial_m"], math.sqrt(0.0025 / 2.0))

    def test_translation_fit_recovers_known_bias(self):
        rows = [
            {
                "point": f"P{index}",
                "corrected_x_m": x + 0.012,
                "corrected_y_m": y - 0.008,
                "physical_x_m": x,
                "physical_y_m": y,
                "marker_height_m": 0.09,
            }
            for index, (x, y) in enumerate(((0.2, 0.2), (0.8, 0.2), (0.5, 0.8)), 1)
        ]
        result = fit_translation_alignment(rows)
        self.assertAlmostEqual(result["offset_x_m"], -0.012)
        self.assertAlmostEqual(result["offset_y_m"], 0.008)
        self.assertAlmostEqual(result["calibration_rmse_m"], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
