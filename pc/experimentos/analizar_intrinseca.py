import argparse
import csv
import json
from pathlib import Path

import matplotlib
import numpy as np
from scipy import stats

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = (
    REPO_ROOT / "paper" / "configuracion_local" / "intrinsic_validation.csv"
)
DEFAULT_CALIBRATION = (
    REPO_ROOT / "paper" / "configuracion_local" / "camera_calibration.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT / "paper" / "resultados" / "procesados" / "intrinseca_off_on"
)
STAGES = ("raw", "parallax")
CONDITIONS = ("off", "on")


def read_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("No hay mediciones de validacion intrinseca")
    for index, row in enumerate(rows, start=1):
        row["plot_label"] = f"P{index}"
        for key, value in list(row.items()):
            if key.endswith("_m") or key.endswith("_pct") or key == "robot_id":
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    pass
    return rows


def error_metrics(values):
    values = np.asarray(values, dtype=float)
    return {
        "count": int(len(values)),
        "mean_error_m": float(np.mean(values)),
        "median_error_m": float(np.median(values)),
        "rmse_m": float(np.sqrt(np.mean(values ** 2))),
        "max_error_m": float(np.max(values)),
    }


def paired_metrics(delta):
    delta = np.asarray(delta, dtype=float)
    count = len(delta)
    mean = float(np.mean(delta))
    std = float(np.std(delta, ddof=1)) if count > 1 else 0.0
    if count > 1:
        margin = float(stats.t.ppf(0.975, count - 1) * std / np.sqrt(count))
    else:
        margin = 0.0
    return {
        "count": int(count),
        "mean": mean,
        "std": std,
        "ci95_low": mean - margin,
        "ci95_high": mean + margin,
    }


def summarize(rows):
    stages = {}
    for stage in STAGES:
        off = np.asarray([row[f"off_{stage}_error_m"] for row in rows], dtype=float)
        on = np.asarray([row[f"on_{stage}_error_m"] for row in rows], dtype=float)
        off_metrics = error_metrics(off)
        on_metrics = error_metrics(on)
        delta = on - off
        stages[stage] = {
            "off": off_metrics,
            "on": on_metrics,
            "paired_on_minus_off_mean_m": float(np.mean(delta)),
            "paired_on_minus_off_median_m": float(np.median(delta)),
            "paired_on_minus_off_stats_m": paired_metrics(delta),
            "on_better_points": int(np.sum(delta < 0)),
            "off_better_points": int(np.sum(delta > 0)),
            "equal_points": int(np.sum(np.isclose(delta, 0.0))),
            "rmse_improvement_pct": float(
                100.0 * (off_metrics["rmse_m"] - on_metrics["rmse_m"])
                / max(off_metrics["rmse_m"], 1e-12)
            ),
        }

    off_detection = np.asarray(
        [row["off_detection_rate_pct"] for row in rows], dtype=float
    )
    on_detection = np.asarray(
        [row["on_detection_rate_pct"] for row in rows], dtype=float
    )
    return {
        "points": len(rows),
        "stages": stages,
        "detection": {
            "off_mean_pct": float(np.mean(off_detection)),
            "on_mean_pct": float(np.mean(on_detection)),
            "paired_improvement_mean_points": float(np.mean(on_detection - off_detection)),
            "paired_improvement_stats_points": paired_metrics(
                on_detection - off_detection
            ),
            "on_better_points": int(np.sum(on_detection > off_detection)),
            "off_better_points": int(np.sum(on_detection < off_detection)),
        },
    }


def write_point_csv(path, rows):
    fields = [
        "plot_label", "point", "physical_x_m", "physical_y_m",
        "off_raw_error_m", "on_raw_error_m", "raw_on_minus_off_m",
        "off_parallax_error_m", "on_parallax_error_m",
        "parallax_on_minus_off_m", "off_detection_rate_pct",
        "on_detection_rate_pct", "detection_on_minus_off_pct",
    ]
    output_rows = []
    for row in rows:
        output_rows.append({
            "plot_label": row["plot_label"],
            "point": row["point"],
            "physical_x_m": row["physical_x_m"],
            "physical_y_m": row["physical_y_m"],
            "off_raw_error_m": row["off_raw_error_m"],
            "on_raw_error_m": row["on_raw_error_m"],
            "raw_on_minus_off_m": row["on_raw_error_m"] - row["off_raw_error_m"],
            "off_parallax_error_m": row["off_parallax_error_m"],
            "on_parallax_error_m": row["on_parallax_error_m"],
            "parallax_on_minus_off_m": (
                row["on_parallax_error_m"] - row["off_parallax_error_m"]
            ),
            "off_detection_rate_pct": row["off_detection_rate_pct"],
            "on_detection_rate_pct": row["on_detection_rate_pct"],
            "detection_on_minus_off_pct": (
                row["on_detection_rate_pct"] - row["off_detection_rate_pct"]
            ),
        })
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output_rows)


def plot_errors(rows, summary, output_path):
    labels = [row["plot_label"] for row in rows]
    x = np.arange(len(rows))
    width = 0.36
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.5), constrained_layout=True)
    colors = ("#757575", "#1565c0")
    for axis, stage, title in zip(
        axes[:2], STAGES, ("Homografia", "Homografia + paralaje")
    ):
        off = [row[f"off_{stage}_error_m"] * 100.0 for row in rows]
        on = [row[f"on_{stage}_error_m"] * 100.0 for row in rows]
        axis.bar(x - width / 2, off, width, label="Intrinseca OFF", color=colors[0])
        axis.bar(x + width / 2, on, width, label="Intrinseca ON", color=colors[1])
        axis.set_xticks(x, labels)
        axis.set_ylabel("Error radial [cm]")
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.25)
        axis.legend(fontsize=8)

    off_detection = [row["off_detection_rate_pct"] for row in rows]
    on_detection = [row["on_detection_rate_pct"] for row in rows]
    axes[2].bar(x - width / 2, off_detection, width, label="OFF", color=colors[0])
    axes[2].bar(x + width / 2, on_detection, width, label="ON", color=colors[1])
    axes[2].set_xticks(x, labels)
    axes[2].set_ylim(0, 105)
    axes[2].set_ylabel("Deteccion [%]")
    axes[2].set_title("Disponibilidad del ArUco")
    axes[2].grid(axis="y", alpha=0.25)
    axes[2].legend(fontsize=8)
    fig.suptitle("Calibracion intrinseca OFF/ON")
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def plot_spatial(rows, output_path):
    fig, axes = plt.subplots(2, 2, figsize=(9, 8.5), constrained_layout=True)
    for axis, stage, condition in zip(
        axes.ravel(),
        ("raw", "raw", "parallax", "parallax"),
        ("off", "on", "off", "on"),
    ):
        truth_x = np.asarray([row["physical_x_m"] for row in rows])
        truth_y = np.asarray([row["physical_y_m"] for row in rows])
        estimate_x = np.asarray([row[f"{condition}_{stage}_x_m"] for row in rows])
        estimate_y = np.asarray([row[f"{condition}_{stage}_y_m"] for row in rows])
        axis.scatter(truth_x, truth_y, marker="x", s=55, color="black", label="Fisico")
        axis.scatter(estimate_x, estimate_y, s=35, color="#1565c0", label="Estimado")
        for row, tx, ty, ex, ey in zip(rows, truth_x, truth_y, estimate_x, estimate_y):
            axis.plot([tx, ex], [ty, ey], color="#1565c0", linewidth=1.2)
            axis.annotate(row["plot_label"], (tx, ty), xytext=(4, 4),
                          textcoords="offset points", fontsize=8)
        stage_title = "Homografia" if stage == "raw" else "Homografia + paralaje"
        axis.set_title(f"{stage_title} | Intrinseca {condition.upper()}")
        axis.set_xlim(0.0, 1.2)
        axis.set_ylim(0.0, 1.2)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", fontsize=8)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def analyze(input_path, calibration_path, output_dir):
    rows = read_rows(input_path)
    summary = summarize(rows)
    calibration = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
    summary["calibration"] = {
        "created_utc": calibration.get("created_utc"),
        "valid_views": calibration.get("valid_views"),
        "rms_reprojection_error_px": calibration.get("rms_reprojection_error"),
        "quality_passed": calibration.get("quality_passed"),
        "focal_ratio": calibration.get("focal_ratio"),
        "normal_spread_deg": calibration.get("normal_spread_deg"),
    }
    summary["method"] = (
        "Paired static positions; sequential OFF/ON frame windows; no XY alignment."
    )
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_point_csv(output_dir / "errors_by_point.csv", rows)
    plot_errors(rows, summary, output_dir / "intrinsic_errors.png")
    plot_spatial(rows, output_dir / "intrinsic_spatial.png")
    return summary


def main():
    parser = argparse.ArgumentParser(description="Analiza calibracion intrinseca OFF/ON")
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()
    summary = analyze(args.input, args.calibration, args.output)
    compact = {
        "raw_rmse_cm": {
            condition: summary["stages"]["raw"][condition]["rmse_m"] * 100.0
            for condition in CONDITIONS
        },
        "parallax_rmse_cm": {
            condition: summary["stages"]["parallax"][condition]["rmse_m"] * 100.0
            for condition in CONDITIONS
        },
        "detection_mean_pct": {
            "off": summary["detection"]["off_mean_pct"],
            "on": summary["detection"]["on_mean_pct"],
        },
    }
    print(json.dumps(compact, indent=2, ensure_ascii=False))
    print(f"Resultados: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
