import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


MAX_RMS_ERROR_PX = 0.80
MAX_FOCAL_RATIO = 1.15
MIN_NORMAL_SPREAD_DEG = 12.0
MIN_CENTER_SPAN = 0.35
MIN_SCALE_RATIO = 1.15


def parse_source(value):
    text = str(value)
    return int(text) if text.isdigit() else text


def object_points(cols, rows, square_m):
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= float(square_m)
    return points


def detect_board(gray, pattern):
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(
            gray,
            pattern,
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        return ok, corners
    ok, corners = cv2.findChessboardCorners(gray, pattern)
    if ok:
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
        )
    return ok, corners


def calibration_quality(matrix, rvecs, image_sets, image_size, rms):
    width, height = image_size
    fx = float(matrix[0, 0])
    fy = float(matrix[1, 1])
    focal_ratio = max(fx, fy) / max(min(fx, fy), 1e-9)

    normals = []
    for rvec in rvecs:
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
        normal = rotation[:, 2]
        if normal[2] < 0:
            normal = -normal
        normals.append(normal / max(np.linalg.norm(normal), 1e-9))

    normal_spread = 0.0
    for index, first in enumerate(normals):
        for second in normals[index + 1:]:
            cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
            normal_spread = max(normal_spread, float(np.degrees(np.arccos(cosine))))

    centers = np.asarray([
        np.mean(np.asarray(corners).reshape(-1, 2), axis=0)
        for corners in image_sets
    ])
    center_span_x = float(np.ptp(centers[:, 0]) / max(width, 1))
    center_span_y = float(np.ptp(centers[:, 1]) / max(height, 1))

    areas = np.asarray([
        cv2.contourArea(cv2.convexHull(np.asarray(corners, dtype=np.float32)))
        for corners in image_sets
    ], dtype=float)
    positive_areas = areas[areas > 0]
    scale_ratio = (
        float(np.max(positive_areas) / np.min(positive_areas))
        if len(positive_areas) else 1.0
    )

    warnings = []
    if float(rms) > MAX_RMS_ERROR_PX:
        warnings.append(
            f"RMS de reproyeccion alto ({float(rms):.3f} px > {MAX_RMS_ERROR_PX:.2f} px)"
        )
    if focal_ratio > MAX_FOCAL_RATIO:
        warnings.append(
            f"Relacion fx/fy poco plausible ({focal_ratio:.3f} > {MAX_FOCAL_RATIO:.2f})"
        )
    if normal_spread < MIN_NORMAL_SPREAD_DEG:
        warnings.append(
            "Poca diversidad angular del tablero "
            f"({normal_spread:.1f} deg < {MIN_NORMAL_SPREAD_DEG:.1f} deg)"
        )
    if center_span_x < MIN_CENTER_SPAN or center_span_y < MIN_CENTER_SPAN:
        warnings.append(
            "Cobertura insuficiente de la imagen "
            f"(span x={center_span_x:.2f}, y={center_span_y:.2f})"
        )
    if scale_ratio < MIN_SCALE_RATIO:
        warnings.append(
            f"Poca variacion de distancia/escala ({scale_ratio:.2f}x < {MIN_SCALE_RATIO:.2f}x)"
        )

    return {
        "quality_passed": not warnings,
        "quality_warnings": warnings,
        "focal_ratio": focal_ratio,
        "normal_spread_deg": normal_spread,
        "center_span_x": center_span_x,
        "center_span_y": center_span_y,
        "scale_ratio": scale_ratio,
        "quality_thresholds": {
            "max_rms_error_px": MAX_RMS_ERROR_PX,
            "max_focal_ratio": MAX_FOCAL_RATIO,
            "min_normal_spread_deg": MIN_NORMAL_SPREAD_DEG,
            "min_center_span": MIN_CENTER_SPAN,
            "min_scale_ratio": MIN_SCALE_RATIO,
        },
    }


def calibrate(images, cols, rows, square_m):
    obj_template = object_points(cols, rows, square_m)
    object_sets = []
    image_sets = []
    image_size = None
    accepted = []

    for image_path in images:
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = detect_board(gray, (cols, rows))
        if not ok:
            continue
        image_size = (gray.shape[1], gray.shape[0])
        object_sets.append(obj_template.copy())
        image_sets.append(corners.astype(np.float32))
        accepted.append(str(image_path))

    if len(object_sets) < 10 or image_size is None:
        raise RuntimeError(f"Se necesitan al menos 10 vistas validas; solo hay {len(object_sets)}")

    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_sets, image_sets, image_size, None, None
    )

    per_view = []
    for obj, detected, rvec, tvec in zip(object_sets, image_sets, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
        residual = detected.reshape(-1, 2) - projected.reshape(-1, 2)
        error = np.sqrt(np.mean(np.sum(residual * residual, axis=1)))
        per_view.append(float(error))

    quality = calibration_quality(matrix, rvecs, image_sets, image_size, rms)

    result = {
        "schema_version": 2,
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "image_width": image_size[0],
        "image_height": image_size[1],
        "checkerboard_inner_cols": cols,
        "checkerboard_inner_rows": rows,
        "square_size_m": square_m,
        "valid_views": len(object_sets),
        "rms_reprojection_error": float(rms),
        "mean_per_view_error_px": float(np.mean(per_view)),
        "max_per_view_error_px": float(np.max(per_view)),
        "per_view_error_px": per_view,
        "camera_matrix": matrix.tolist(),
        "dist_coeffs": distortion.reshape(-1).tolist(),
        "accepted_images": accepted,
    }
    result.update(quality)
    return result


def capture_images(source, output_dir, cols, rows, target, auto_interval):
    output_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(parse_source(source))
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la fuente: {source}")

    count = len(list(output_dir.glob("calib_*.png")))
    last_auto = 0.0
    print("ESPACIO: capturar | C: calibrar | Q/ESC: terminar")
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = detect_board(gray, (cols, rows))
            view = frame.copy()
            if found:
                cv2.drawChessboardCorners(view, (cols, rows), corners, found)
            cv2.putText(
                view,
                f"capturas={count}/{target} tablero={'OK' if found else 'NO'}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if found else (0, 0, 255),
                2,
            )
            cv2.imshow("Calibracion intrinseca", view)
            key = cv2.waitKey(1) & 0xFF
            auto_ready = auto_interval > 0 and (time.time() - last_auto) >= auto_interval
            if found and (key == ord(" ") or auto_ready):
                count += 1
                path = output_dir / f"calib_{count:03d}.png"
                cv2.imwrite(str(path), frame)
                last_auto = time.time()
                print(f"Capturada: {path}")
            if key in (27, ord("q"), ord("Q"), ord("c"), ord("C")) or count >= target:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def build_parser():
    parser = argparse.ArgumentParser(description="Calibracion intrinseca de la camara cenital")
    parser.add_argument("--source", default="http://raspberry-5.local:5000/video")
    parser.add_argument("--cols", type=int, default=9, help="Esquinas internas horizontales")
    parser.add_argument("--rows", type=int, default=6, help="Esquinas internas verticales")
    parser.add_argument("--square-mm", type=float, default=24.0)
    parser.add_argument("--captures", type=int, default=25)
    parser.add_argument("--auto-interval", type=float, default=0.0)
    parser.add_argument("--images", type=Path, default=Path("paper/resultados/raw/calibracion/images"))
    parser.add_argument("--output", type=Path, default=Path("paper/configuracion_local/camera_calibration.json"))
    parser.add_argument("--skip-capture", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    if not args.skip_capture:
        capture_images(
            args.source,
            args.images,
            args.cols,
            args.rows,
            args.captures,
            args.auto_interval,
        )
    images = sorted(args.images.glob("calib_*.png"))
    result = calibrate(images, args.cols, args.rows, args.square_mm / 1000.0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Calibracion guardada en: {args.output.resolve()}")
    print(f"RMS de reproyeccion: {result['rms_reprojection_error']:.4f} px")
    print(f"Vistas validas: {result['valid_views']}")
    print(
        "Diversidad: "
        f"angulos={result['normal_spread_deg']:.1f} deg | "
        f"cobertura={result['center_span_x']:.2f}x{result['center_span_y']:.2f} | "
        f"escala={result['scale_ratio']:.2f}x | "
        f"fx/fy={result['focal_ratio']:.3f}"
    )
    if result["quality_passed"]:
        print("CALIDAD: OK")
    else:
        print("CALIDAD: NO APTA")
        for warning in result["quality_warnings"]:
            print(f"  - {warning}")


if __name__ == "__main__":
    main()
