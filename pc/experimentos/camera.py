import json
from pathlib import Path

import cv2
import numpy as np


def load_camera_calibration(path):
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    matrix = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    return {
        "path": str(path.resolve()),
        "camera_matrix": matrix,
        "dist_coeffs": distortion,
        "image_width": int(data["image_width"]),
        "image_height": int(data["image_height"]),
        "rms_reprojection_error": float(data.get("rms_reprojection_error", float("nan"))),
        "raw": data,
    }


def scaled_camera_matrix(calibration, width, height):
    matrix = calibration["camera_matrix"].copy()
    sx = float(width) / max(float(calibration["image_width"]), 1.0)
    sy = float(height) / max(float(calibration["image_height"]), 1.0)
    matrix[0, 0] *= sx
    matrix[0, 2] *= sx
    matrix[1, 1] *= sy
    matrix[1, 2] *= sy
    return matrix


def undistort_frame(frame, calibration):
    height, width = frame.shape[:2]
    matrix = scaled_camera_matrix(calibration, width, height)
    corrected = cv2.undistort(frame, matrix, calibration["dist_coeffs"], None, matrix)
    return corrected, matrix

