import sys
import unittest
from pathlib import Path

import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parent
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from snapshot_controller import merge_aruco_detections


def marker(value):
    return np.full((1, 4, 2), float(value), dtype=np.float32)


class ArucoMergeTests(unittest.TestCase):
    def test_adds_only_missing_fallback_markers(self):
        corners, ids = merge_aruco_detections(
            [marker(4), marker(5)],
            np.asarray([[4], [5]], dtype=np.int32),
            [marker(5), marker(6)],
            np.asarray([[5], [6]], dtype=np.int32),
        )
        self.assertEqual(ids.flatten().tolist(), [4, 5, 6])
        self.assertEqual(len(corners), 3)
        np.testing.assert_array_equal(corners[1], marker(5))

    def test_handles_empty_primary_detection(self):
        corners, ids = merge_aruco_detections(
            [], None, [marker(7)], np.asarray([[7]], dtype=np.int32)
        )
        self.assertEqual(ids.flatten().tolist(), [7])
        self.assertEqual(len(corners), 1)


if __name__ == "__main__":
    unittest.main()
