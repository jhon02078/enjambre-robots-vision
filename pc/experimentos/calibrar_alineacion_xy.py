import argparse
import csv
import json
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = (
    REPO_ROOT / "paper" / "configuracion_local" / "parallax_measurements.csv"
)
DEFAULT_OUTPUT = (
    REPO_ROOT / "paper" / "configuracion_local" / "localization_alignment.json"
)


def fit_translation_alignment(rows, workspace_width_m=1.20, workspace_height_m=1.20):
    if len(rows) < 3:
        raise ValueError("Se requieren al menos tres puntos de calibracion")
    offsets_x = []
    offsets_y = []
    marker_heights = []
    points = []
    for row in rows:
        corrected_x = float(row["corrected_x_m"])
        corrected_y = float(row["corrected_y_m"])
        physical_x = float(row["physical_x_m"])
        physical_y = float(row["physical_y_m"])
        offsets_x.append(physical_x - corrected_x)
        offsets_y.append(physical_y - corrected_y)
        marker_heights.append(float(row["marker_height_m"]))
        points.append(row["point"])

    offset_x = sum(offsets_x) / len(offsets_x)
    offset_y = sum(offsets_y) / len(offsets_y)
    errors = []
    for row in rows:
        aligned_x = float(row["corrected_x_m"]) + offset_x
        aligned_y = float(row["corrected_y_m"]) + offset_y
        errors.append(math.hypot(
            aligned_x - float(row["physical_x_m"]),
            aligned_y - float(row["physical_y_m"]),
        ))

    return {
        "version": 1,
        "method": "translation_bias_after_parallax",
        "enabled": True,
        "offset_x_m": offset_x,
        "offset_y_m": offset_y,
        "marker_height_m": sum(marker_heights) / len(marker_heights),
        "workspace_width_m": float(workspace_width_m),
        "workspace_height_m": float(workspace_height_m),
        "calibration_points": points,
        "calibration_rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "calibration_max_error_m": max(errors),
    }


def calibrate(input_path, output_path, workspace_width_m=1.20, workspace_height_m=1.20):
    with Path(input_path).open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    alignment = fit_translation_alignment(rows, workspace_width_m, workspace_height_m)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(alignment, indent=2), encoding="utf-8")
    return alignment


def main():
    parser = argparse.ArgumentParser(description="Ajusta una alineacion XY posterior al paralaje")
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--width", type=float, default=1.20)
    parser.add_argument("--height", type=float, default=1.20)
    args = parser.parse_args()
    result = calibrate(args.input, args.output, args.width, args.height)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"Alineacion guardada en: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
