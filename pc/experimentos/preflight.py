import argparse
import json
import socket
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np


PC_DIR = Path(__file__).resolve().parents[1]
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

from camera_stream import open_camera_stream
from experimentos.camera import load_camera_calibration, undistort_frame
from experimentos.localization import (
    apply_localization_alignment,
    load_localization_alignment,
)


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(
    ARUCO_DICT,
    cv2.aruco.DetectorParameters(),
)
WORKSPACE_IDS = (4, 5, 6, 7)
DEFAULT_ALIGNMENT = (
    PC_DIR.parent / "paper" / "configuracion_local" / "localization_alignment.json"
)
DEFAULT_CAMERA_CALIBRATION = (
    PC_DIR.parent / "paper" / "configuracion_local" / "camera_calibration.json"
)


def camera_broadcast(url):
    try:
        host = urlparse(url).hostname
        parts = host.split(".") if host else []
        if len(parts) == 4 and all(0 <= int(part) <= 255 for part in parts):
            return ".".join(parts[:3] + ["255"])
    except (TypeError, ValueError):
        pass
    return None


def cached_robot_endpoints():
    cache_path = PC_DIR / "robots_cache.json"
    if not cache_path.exists():
        return set()
    try:
        data = json.loads(cache_path.read_text(encoding="utf-8"))
        return {
            (str(item["ip"]), 37030)
            for item in data.get("robots", {}).values()
            if item.get("ip")
        }
    except (OSError, ValueError, TypeError, KeyError):
        return set()


def discover_robots(camera_url, duration_s=2.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", 0))
    sock.settimeout(0.15)
    endpoints = {("255.255.255.255", 37030)}
    endpoints.update(cached_robot_endpoints())
    subnet = camera_broadcast(camera_url)
    if subnet:
        endpoints.add((subnet, 37030))
    robots = {}
    deadline = time.monotonic() + float(duration_s)
    try:
        while time.monotonic() < deadline:
            for endpoint in endpoints:
                try:
                    sock.sendto(b"DISCOVER_ROBOTS", endpoint)
                except OSError:
                    pass
            listen_until = min(deadline, time.monotonic() + 0.35)
            while time.monotonic() < listen_until:
                try:
                    data, address = sock.recvfrom(256)
                except socket.timeout:
                    break
                text = data.decode(errors="ignore").strip()
                if not text.startswith("ROBOT_HERE"):
                    continue
                values = {}
                for token in text.split()[1:]:
                    if "=" in token:
                        key, value = token.split("=", 1)
                        values[key] = value
                try:
                    robot_id = int(values["ID"])
                    port = int(values.get("CMDPORT", "44444"))
                except (KeyError, ValueError):
                    continue
                robots[robot_id] = {"ip": address[0], "port": port}
    finally:
        sock.close()
    return robots


def inspect_camera(
    url,
    frame_count,
    width_m,
    height_m,
    marker_height_m,
    alignment=None,
    calibration=None,
):
    stream = open_camera_stream(url)
    detections = Counter()
    centers = defaultdict(list)
    marker_corners = defaultdict(list)
    shapes = Counter()
    calibrated_camera_matrix = None
    started = time.perf_counter()
    try:
        for _ in range(int(frame_count)):
            frame = stream.read()
            if calibration is not None:
                frame, calibrated_camera_matrix = undistort_frame(frame, calibration)
            shapes[str(tuple(frame.shape))] += 1
            corners, ids, _ = ARUCO_DETECTOR.detectMarkers(frame)
            if ids is None:
                continue
            for marker_id, corner in zip(ids.flatten().tolist(), corners):
                marker_id = int(marker_id)
                center = np.mean(corner[0], axis=0)
                detections[marker_id] += 1
                centers[marker_id].append([float(center[0]), float(center[1])])
                marker_corners[marker_id].append(
                    np.asarray(corner[0], dtype=np.float32)
                )
    finally:
        stream.release()
    elapsed = time.perf_counter() - started

    mean_centers = {
        marker_id: np.mean(np.asarray(samples), axis=0)
        for marker_id, samples in centers.items()
        if samples
    }
    mean_corners = {
        marker_id: np.mean(np.asarray(samples), axis=0)
        for marker_id, samples in marker_corners.items()
        if samples
    }
    poses = {}
    corrected_poses = {}
    aligned_poses = {}
    camera_pose = None
    homography_available = all(marker_id in mean_centers for marker_id in WORKSPACE_IDS)
    if homography_available:
        image_points = np.asarray(
            [mean_centers[marker_id] for marker_id in WORKSPACE_IDS],
            dtype=np.float32,
        )
        world_points = np.asarray(
            [[0.0, 0.0], [width_m, 0.0], [width_m, height_m], [0.0, height_m]],
            dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(image_points, world_points)
        shape_text = next(iter(shapes), "")
        shape_values = [int(value) for value in shape_text.strip("()").split(",") if value.strip()]
        if len(shape_values) >= 2:
            image_height, image_width = shape_values[:2]
            if calibrated_camera_matrix is None:
                focal = 0.95 * image_width
                camera_matrix = np.asarray(
                    [[focal, 0.0, image_width / 2.0],
                     [0.0, focal, image_height / 2.0],
                     [0.0, 0.0, 1.0]],
                    dtype=np.float32,
                )
            else:
                camera_matrix = np.asarray(calibrated_camera_matrix, dtype=np.float32)
            object_points = np.asarray(
                [[0.0, 0.0, 0.0], [width_m, 0.0, 0.0],
                 [width_m, height_m, 0.0], [0.0, height_m, 0.0]],
                dtype=np.float32,
            )
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                np.zeros((5, 1), dtype=np.float32),
                flags=cv2.SOLVEPNP_IPPE,
            )
            if ok:
                rotation, _ = cv2.Rodrigues(rvec)
                center = (-rotation.T @ tvec).reshape(-1)
                if center[2] >= 0.40:
                    camera_pose = [float(value) for value in center]
        for robot_id in (1, 2, 3, 10):
            if robot_id not in mean_centers:
                continue
            point = np.asarray([[mean_centers[robot_id]]], dtype=np.float32)
            world = cv2.perspectiveTransform(point, homography)[0, 0]
            robot_corner = mean_corners[robot_id]
            heading_points = cv2.perspectiveTransform(
                np.asarray([[robot_corner[0], robot_corner[1]]], dtype=np.float32),
                homography,
            )[0]
            yaw_rad = float(
                np.arctan2(
                    heading_points[1][1] - heading_points[0][1],
                    heading_points[1][0] - heading_points[0][0],
                )
            )
            yaw_deg = float(np.degrees(yaw_rad))
            poses[robot_id] = {
                "x_m": float(world[0]),
                "y_m": float(world[1]),
                "yaw_rad": yaw_rad,
                "yaw_deg": yaw_deg,
            }
            if camera_pose and camera_pose[2] > marker_height_m + 0.1:
                scale = (camera_pose[2] - marker_height_m) / camera_pose[2]
                corrected_poses[robot_id] = {
                    "x_m": camera_pose[0] + (float(world[0]) - camera_pose[0]) * scale,
                    "y_m": camera_pose[1] + (float(world[1]) - camera_pose[1]) * scale,
                    "yaw_rad": yaw_rad,
                    "yaw_deg": yaw_deg,
                }
                aligned_x, aligned_y = apply_localization_alignment(
                    corrected_poses[robot_id]["x_m"],
                    corrected_poses[robot_id]["y_m"],
                    alignment,
                    enabled=alignment is not None,
                )
                aligned_poses[robot_id] = {
                    "x_m": aligned_x,
                    "y_m": aligned_y,
                    "yaw_rad": yaw_rad,
                    "yaw_deg": yaw_deg,
                }

    return {
        "intrinsic_calibration_enabled": calibration is not None,
        "intrinsic_calibration_path": (
            calibration.get("path") if calibration is not None else None
        ),
        "frames": int(frame_count),
        "elapsed_s": elapsed,
        "camera_fps": float(frame_count) / max(elapsed, 1e-9),
        "frame_shapes": dict(shapes),
        "detections": {str(key): value for key, value in sorted(detections.items())},
        "detection_rate_pct": {
            str(key): 100.0 * value / max(int(frame_count), 1)
            for key, value in sorted(detections.items())
        },
        "homography_available": homography_available,
        "fallback_camera_pose_m": camera_pose,
        "raw_robot_poses_m": {str(key): value for key, value in sorted(poses.items())},
        "corrected_robot_poses_m": {
            str(key): value for key, value in sorted(corrected_poses.items())
        },
        "aligned_robot_poses_m": {
            str(key): value for key, value in sorted(aligned_poses.items())
        },
    }


def main():
    parser = argparse.ArgumentParser(description="Preflight sin movimiento para las pruebas fisicas")
    parser.add_argument("--source", default="http://raspberry-5.local:5000/video")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--width", type=float, default=1.20)
    parser.add_argument("--height", type=float, default=1.20)
    parser.add_argument("--marker-height", type=float, default=0.09)
    parser.add_argument("--robots", type=int, nargs="+", default=[2, 3])
    parser.add_argument("--alignment", default=str(DEFAULT_ALIGNMENT))
    parser.add_argument("--disable-alignment", action="store_true")
    parser.add_argument(
        "--camera-calibration", default=str(DEFAULT_CAMERA_CALIBRATION)
    )
    parser.add_argument("--enable-camera-calibration", action="store_true")
    args = parser.parse_args()

    alignment = None
    alignment_error = None
    if not args.disable_alignment:
        try:
            alignment = load_localization_alignment(args.alignment)
        except Exception as exc:
            alignment_error = str(exc)

    camera_calibration = None
    if args.enable_camera_calibration:
        camera_calibration = load_camera_calibration(args.camera_calibration)
        if camera_calibration["raw"].get("quality_passed") is False:
            raise RuntimeError(
                "La calibracion intrinseca no paso los controles de calidad: "
                + "; ".join(camera_calibration["raw"].get("quality_warnings", []))
            )

    report = {
        "camera": inspect_camera(
            args.source,
            args.frames,
            args.width,
            args.height,
            args.marker_height,
            alignment=alignment,
            calibration=camera_calibration,
        ),
        "network": discover_robots(args.source),
        "alignment": alignment,
        "alignment_error": alignment_error,
        "camera_calibration": (
            {
                "path": camera_calibration["path"],
                "rms_reprojection_error": camera_calibration["rms_reprojection_error"],
                "quality_passed": camera_calibration["raw"].get("quality_passed"),
            }
            if camera_calibration is not None else None
        ),
    }
    required_markers = {4, 5, 6, 7, *args.robots}
    detected_markers = {int(value) for value in report["camera"]["detections"]}
    discovered_robots = {int(value) for value in report["network"]}
    report["checks"] = {
        "required_markers": sorted(required_markers),
        "missing_markers": sorted(required_markers - detected_markers),
        "required_robots": sorted(args.robots),
        "robots_without_udp": sorted(set(args.robots) - discovered_robots),
        "ready_without_motion": (
            required_markers.issubset(detected_markers)
            and set(args.robots).issubset(discovered_robots)
            and report["camera"]["homography_available"]
        ),
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
