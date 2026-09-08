import unittest

import cv2
import numpy as np

from calibrar_camara import calibration_quality


def corner_cloud(center_x, center_y, scale=1.0):
    x, y = np.meshgrid(np.arange(9), np.arange(6))
    points = np.column_stack((x.ravel(), y.ravel())).astype(np.float32)
    points -= np.mean(points, axis=0)
    points *= 18.0 * scale
    points += np.asarray([center_x, center_y], dtype=np.float32)
    return points.reshape(-1, 1, 2)


class CalibrationQualityTests(unittest.TestCase):
    def setUp(self):
        self.matrix = np.asarray([
            [1000.0, 0.0, 640.0],
            [0.0, 1010.0, 480.0],
            [0.0, 0.0, 1.0],
        ])

    def test_rejects_parallel_views(self):
        rvecs = [np.zeros((3, 1), dtype=float) for _ in range(4)]
        corners = [
            corner_cloud(250, 220),
            corner_cloud(1030, 220),
            corner_cloud(250, 740),
            corner_cloud(1030, 740),
        ]
        result = calibration_quality(self.matrix, rvecs, corners, (1280, 960), 0.2)
        self.assertFalse(result["quality_passed"])
        self.assertLess(result["normal_spread_deg"], 1.0)

    def test_accepts_varied_well_distributed_views(self):
        angles = ((0, 0), (18, 0), (-18, 0), (0, 18), (0, -18))
        rvecs = []
        for x_deg, y_deg in angles:
            rotation_x, _ = cv2.Rodrigues(
                np.asarray([np.radians(x_deg), 0.0, 0.0], dtype=float)
            )
            rotation_y, _ = cv2.Rodrigues(
                np.asarray([0.0, np.radians(y_deg), 0.0], dtype=float)
            )
            rvec, _ = cv2.Rodrigues(rotation_y @ rotation_x)
            rvecs.append(rvec)
        corners = [
            corner_cloud(220, 180, 0.90),
            corner_cloud(1060, 190, 1.00),
            corner_cloud(230, 770, 1.05),
            corner_cloud(1050, 760, 1.12),
            corner_cloud(640, 480, 1.20),
        ]
        result = calibration_quality(self.matrix, rvecs, corners, (1280, 960), 0.2)
        self.assertTrue(result["quality_passed"], result["quality_warnings"])


if __name__ == "__main__":
    unittest.main()
