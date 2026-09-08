import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION = (
    REPO_ROOT / "paper" / "configuracion_local" / "parallax_measurements.csv"
)
DEFAULT_VALIDATION = (
    REPO_ROOT / "paper" / "configuracion_local" / "localization_validation.csv"
)
DEFAULT_ALIGNMENT = (
    REPO_ROOT / "paper" / "configuracion_local" / "localization_alignment.json"
)
DEFAULT_OUTPUT = REPO_ROOT / "paper" / "resultados" / "procesados" / "localizacion_grid"


STAGES = (
    ("homography", "raw_x_m", "raw_y_m", "Homografia"),
    ("parallax", "corrected_x_m", "corrected_y_m", "Paralaje"),
    ("aligned", "aligned_x_m", "aligned_y_m", "Alineacion XY"),
)


def read_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(row, key):
    return float(row[key])


def prepare_validation_rows(rows):
    prepared = []
    for row in rows:
        if str(row.get("include_final", "0")).strip() not in {"1", "true", "True"}:
            continue
        item = dict(row)
        for key in (
            "command_x_m", "command_y_m", "raw_x_m", "raw_y_m",
            "corrected_x_m", "corrected_y_m", "aligned_x_m", "aligned_y_m",
            "physical_x_m", "physical_y_m",
        ):
            item[key] = as_float(row, key)
        item["plot_label"] = f"P{len(prepared) + 1}"
        prepared.append(item)
    if not prepared:
        raise ValueError("No hay filas de validacion marcadas con include_final=1")
    return prepared


def prepare_calibration_rows(rows, alignment):
    dx = float(alignment["offset_x_m"])
    dy = float(alignment["offset_y_m"])
    prepared = []
    for row in rows:
        item = dict(row)
        for key in (
            "command_x_m", "command_y_m", "raw_x_m", "raw_y_m",
            "corrected_x_m", "corrected_y_m", "physical_x_m", "physical_y_m",
        ):
            item[key] = as_float(row, key)
        item["aligned_x_m"] = item["corrected_x_m"] + dx
        item["aligned_y_m"] = item["corrected_y_m"] + dy
        prepared.append(item)
    return prepared


def stage_metrics(rows, x_key, y_key):
    errors_x = np.asarray(
        [float(row[x_key]) - float(row["physical_x_m"]) for row in rows],
        dtype=float,
    )
    errors_y = np.asarray(
        [float(row[y_key]) - float(row["physical_y_m"]) for row in rows],
        dtype=float,
    )
    radial = np.hypot(errors_x, errors_y)
    return {
        "count": int(len(radial)),
        "mean_error_m": float(np.mean(radial)),
        "median_error_m": float(np.median(radial)),
        "rmse_radial_m": float(np.sqrt(np.mean(radial ** 2))),
        "max_error_m": float(np.max(radial)),
        "bias_x_m": float(np.mean(errors_x)),
        "bias_y_m": float(np.mean(errors_y)),
        "rmse_x_m": float(np.sqrt(np.mean(errors_x ** 2))),
        "rmse_y_m": float(np.sqrt(np.mean(errors_y ** 2))),
    }


def positioning_metrics(rows):
    radial = np.asarray([
        math.hypot(
            float(row["physical_x_m"]) - float(row["command_x_m"]),
            float(row["physical_y_m"]) - float(row["command_y_m"]),
        )
        for row in rows
    ])
    return {
        "count": int(len(radial)),
        "mean_error_m": float(np.mean(radial)),
        "rmse_radial_m": float(np.sqrt(np.mean(radial ** 2))),
        "max_error_m": float(np.max(radial)),
    }


def per_point_errors(rows):
    result = []
    for row in rows:
        item = {
            "point": row["point"],
            "plot_label": row["plot_label"],
            "physical_x_m": float(row["physical_x_m"]),
            "physical_y_m": float(row["physical_y_m"]),
            "command_x_m": float(row["command_x_m"]),
            "command_y_m": float(row["command_y_m"]),
        }
        for stage, x_key, y_key, _ in STAGES:
            item[f"{stage}_error_m"] = math.hypot(
                float(row[x_key]) - float(row["physical_x_m"]),
                float(row[y_key]) - float(row["physical_y_m"]),
            )
        item["positioning_error_m"] = math.hypot(
            float(row["physical_x_m"]) - float(row["command_x_m"]),
            float(row["physical_y_m"]) - float(row["command_y_m"]),
        )
        result.append(item)
    return result


def write_point_csv(path, rows):
    fields = list(rows[0])
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_spatial(rows, output_path):
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.5), constrained_layout=True)
    colors = ("#c62828", "#ef6c00", "#1565c0")
    for axis, (stage, x_key, y_key, label), color in zip(axes, STAGES, colors):
        truth_x = np.asarray([row["physical_x_m"] for row in rows])
        truth_y = np.asarray([row["physical_y_m"] for row in rows])
        estimate_x = np.asarray([row[x_key] for row in rows])
        estimate_y = np.asarray([row[y_key] for row in rows])
        axis.scatter(truth_x, truth_y, marker="x", s=55, color="black", label="Fisico")
        axis.scatter(estimate_x, estimate_y, s=35, color=color, label=label)
        for row, tx, ty, ex, ey in zip(rows, truth_x, truth_y, estimate_x, estimate_y):
            axis.plot([tx, ex], [ty, ey], color=color, linewidth=1.3)
            axis.annotate(
                row["plot_label"],
                (tx, ty),
                fontsize=8,
                xytext=(4, 4),
                textcoords="offset points",
            )
        axis.set_title(label)
        axis.set_xlim(0.0, 1.2)
        axis.set_ylim(0.0, 1.2)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.legend(loc="upper right", fontsize=8)
    fig.suptitle("Error espacial de localizacion")
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def plot_errors(point_rows, metrics, output_path):
    labels = [row["plot_label"] for row in point_rows]
    x = np.arange(len(labels))
    width = 0.24
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6), constrained_layout=True)
    colors = {"homography": "#c62828", "parallax": "#ef6c00", "aligned": "#1565c0"}
    for index, (stage, _, _, label) in enumerate(STAGES):
        values_cm = [row[f"{stage}_error_m"] * 100.0 for row in point_rows]
        axes[0].bar(x + (index - 1) * width, values_cm, width, label=label, color=colors[stage])
    axes[0].set_xticks(x, labels, rotation=18, ha="right")
    axes[0].set_ylabel("Error radial [cm]")
    axes[0].set_title("Error por punto")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()

    stage_names = [item[0] for item in STAGES]
    stage_labels = [item[3] for item in STAGES]
    rmse_cm = [metrics[name]["rmse_radial_m"] * 100.0 for name in stage_names]
    max_cm = [metrics[name]["max_error_m"] * 100.0 for name in stage_names]
    sx = np.arange(len(stage_names))
    axes[1].bar(sx - 0.18, rmse_cm, 0.36, label="RMSE", color="#2e7d32")
    axes[1].bar(sx + 0.18, max_cm, 0.36, label="Maximo", color="#6a1b9a")
    axes[1].set_xticks(sx, stage_labels)
    axes[1].set_ylabel("Error [cm]")
    axes[1].set_title("Resumen de validacion")
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def analyze(calibration_path, validation_path, alignment_path, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    alignment = json.loads(Path(alignment_path).read_text(encoding="utf-8"))
    calibration_rows = prepare_calibration_rows(read_rows(calibration_path), alignment)
    validation_rows = prepare_validation_rows(read_rows(validation_path))
    validation_metrics = {
        stage: stage_metrics(validation_rows, x_key, y_key)
        for stage, x_key, y_key, _ in STAGES
    }
    calibration_metrics = {
        stage: stage_metrics(calibration_rows, x_key, y_key)
        for stage, x_key, y_key, _ in STAGES
    }
    point_rows = per_point_errors(validation_rows)
    summary = {
        "calibration_points": len(calibration_rows),
        "validation_points": len(validation_rows),
        "alignment": alignment,
        "calibration_metrics": calibration_metrics,
        "validation_metrics": validation_metrics,
        "positioning_metrics_all_validation_points": positioning_metrics(validation_rows),
        "point_map": {
            row["plot_label"]: {
                "point": row["point"],
                "physical_x_m": row["physical_x_m"],
                "physical_y_m": row["physical_y_m"],
            }
            for row in validation_rows
        },
        "note": "V_09_06_FILTER_DIAG is valid for localization but records a pre-fix positioning failure.",
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_point_csv(output_dir / "errors_by_point.csv", point_rows)
    plot_spatial(validation_rows, output_dir / "localization_spatial.png")
    plot_errors(point_rows, validation_metrics, output_dir / "localization_errors.png")
    return summary


def main():
    parser = argparse.ArgumentParser(description="Analiza la validacion de localizacion sobre la cuadricula")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--validation", default=str(DEFAULT_VALIDATION))
    parser.add_argument("--alignment", default=str(DEFAULT_ALIGNMENT))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()
    summary = analyze(args.calibration, args.validation, args.alignment, args.output)
    compact = {
        stage: {
            "rmse_cm": values["rmse_radial_m"] * 100.0,
            "max_cm": values["max_error_m"] * 100.0,
        }
        for stage, values in summary["validation_metrics"].items()
    }
    print(json.dumps(compact, indent=2, ensure_ascii=False))
    print(f"Resultados: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
