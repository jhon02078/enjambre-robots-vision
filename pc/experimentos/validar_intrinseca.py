import argparse
import csv
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1]
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

from experimentos.camera import load_camera_calibration
from preflight import DEFAULT_CAMERA_CALIBRATION, inspect_camera


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = (
    REPO_ROOT / "paper" / "configuracion_local" / "intrinsic_validation.csv"
)


def robot_pose(report, robot_id, stage):
    poses = report[f"{stage}_robot_poses_m"]
    key = str(int(robot_id))
    if key not in poses:
        raise RuntimeError(
            f"R{robot_id} no fue detectado en la captura de {stage}"
        )
    return poses[key]


def radial_error(pose, physical_x, physical_y):
    return math.hypot(
        float(pose["x_m"]) - float(physical_x),
        float(pose["y_m"]) - float(physical_y),
    )


def build_record(point, robot_id, physical_x, physical_y, off_report, on_report):
    off_raw = robot_pose(off_report, robot_id, "raw")
    on_raw = robot_pose(on_report, robot_id, "raw")
    off_parallax = robot_pose(off_report, robot_id, "corrected")
    on_parallax = robot_pose(on_report, robot_id, "corrected")
    detection_key = str(int(robot_id))
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "point": str(point),
        "robot_id": int(robot_id),
        "physical_x_m": float(physical_x),
        "physical_y_m": float(physical_y),
        "off_raw_x_m": off_raw["x_m"],
        "off_raw_y_m": off_raw["y_m"],
        "off_raw_error_m": radial_error(off_raw, physical_x, physical_y),
        "on_raw_x_m": on_raw["x_m"],
        "on_raw_y_m": on_raw["y_m"],
        "on_raw_error_m": radial_error(on_raw, physical_x, physical_y),
        "off_parallax_x_m": off_parallax["x_m"],
        "off_parallax_y_m": off_parallax["y_m"],
        "off_parallax_error_m": radial_error(off_parallax, physical_x, physical_y),
        "on_parallax_x_m": on_parallax["x_m"],
        "on_parallax_y_m": on_parallax["y_m"],
        "on_parallax_error_m": radial_error(on_parallax, physical_x, physical_y),
        "off_detection_rate_pct": off_report["detection_rate_pct"].get(detection_key, 0.0),
        "on_detection_rate_pct": on_report["detection_rate_pct"].get(detection_key, 0.0),
        "off_camera_pose_m": json.dumps(off_report["fallback_camera_pose_m"]),
        "on_camera_pose_m": json.dumps(on_report["fallback_camera_pose_m"]),
    }


def append_record(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(record))
        if not exists:
            writer.writeheader()
        writer.writerow(record)


def main():
    parser = argparse.ArgumentParser(
        description="Compara localizacion intrinseca OFF/ON en una posicion estatica"
    )
    parser.add_argument("--source", default="http://raspberry-5.local:5000/video")
    parser.add_argument("--robot", type=int, required=True)
    parser.add_argument("--point", required=True)
    parser.add_argument("--physical-x", type=float, required=True)
    parser.add_argument("--physical-y", type=float, required=True)
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--width", type=float, default=1.20)
    parser.add_argument("--height", type=float, default=1.20)
    parser.add_argument("--marker-height", type=float, default=0.09)
    parser.add_argument("--calibration", default=str(DEFAULT_CAMERA_CALIBRATION))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    calibration = load_camera_calibration(args.calibration)
    if calibration["raw"].get("quality_passed") is not True:
        raise RuntimeError("La calibracion no esta marcada con CALIDAD: OK")

    common = (
        args.source,
        args.frames,
        args.width,
        args.height,
        args.marker_height,
    )
    off_report = inspect_camera(*common, alignment=None, calibration=None)
    on_report = inspect_camera(*common, alignment=None, calibration=calibration)
    record = build_record(
        args.point,
        args.robot,
        args.physical_x,
        args.physical_y,
        off_report,
        on_report,
    )
    append_record(args.output, record)
    print(json.dumps(record, indent=2, ensure_ascii=False))
    print(f"Resultado agregado a: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
