import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from .analizar_experimento import summarize_run
except ImportError:
    from analizar_experimento import summarize_run


def as_number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def flatten_summary(summary):
    robots = summary.get("robots", [])
    return {
        "run_dir": summary.get("run_dir"),
        "scenario": summary.get("scenario"),
        "condition": summary.get("condition"),
        "replicate": summary.get("replicate"),
        "controller_mode": summary.get("controller_mode"),
        "requested_delay_ms": as_number(summary.get("requested_delay_ms", 0.0)),
        "duration_s": as_number(summary.get("duration_s")),
        "task_duration_s": as_number(summary.get("task_duration_s")),
        "task_completed": 1.0 if summary.get("task_completed") else 0.0,
        "trial_valid": 1.0 if summary.get("trial_valid", True) else 0.0,
        "outcome": summary.get("outcome"),
        "stop_reason": summary.get("stop_reason"),
        "minimum_pairwise_distance_m": as_number(summary.get("minimum_pairwise_distance_m")),
        "collision_threshold_episodes": as_number(summary.get("collision_threshold_episodes", 0)),
        "ack_rate_pct": as_number(summary.get("ack_rate_pct")),
        "rtt_mean_ms": as_number(summary.get("rtt_mean_ms")),
        "rtt_p95_ms": as_number(summary.get("rtt_p95_ms")),
        "actual_delay_mean_ms": as_number(summary.get("actual_delay_mean_ms")),
        "actual_delay_p95_ms": as_number(summary.get("actual_delay_p95_ms")),
        "vision_processing_mean_ms": as_number(summary.get("vision_processing_mean_ms")),
        "frame_wait_mean_ms": as_number(summary.get("frame_wait_mean_ms")),
        "homography_valid_rate_pct": as_number(summary.get("homography_valid_rate_pct")),
        "detection_rate_pct": float(np.nanmean([as_number(row.get("detection_rate_pct")) for row in robots])) if robots else float("nan"),
        "path_length_m": float(np.nansum([as_number(row.get("path_length_m")) for row in robots])) if robots else float("nan"),
        "distance_rmse_m": float(np.nanmean([as_number(row.get("distance_rmse_m")) for row in robots])) if robots else float("nan"),
        "heading_mae_deg": float(np.nanmean([as_number(row.get("heading_mae_deg")) for row in robots])) if robots else float("nan"),
        "control_effort_abs_pct_s": float(np.nansum([as_number(row.get("control_effort_abs_pct_s")) for row in robots])) if robots else float("nan"),
        "state_transitions": float(np.nansum([as_number(row.get("state_transitions")) for row in robots])) if robots else float("nan"),
        "angular_sign_changes": float(np.nansum([as_number(row.get("angular_sign_changes")) for row in robots])) if robots else float("nan"),
        "targets_reached": float(np.nansum([as_number(row.get("target_reached_events")) for row in robots])) if robots else 0.0,
    }


def discover_runs(root):
    return sorted(path.parent for path in Path(root).rglob("manifest.json") if path.parent.name != "processed")


def load_selected_runs(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    base_dir = Path(path).resolve().parents[2]
    run_dirs = []
    for value in payload.get("runs", []):
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = base_dir / candidate
        run_dirs.append(candidate.resolve())
    return run_dirs


def confidence_interval(values):
    values = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    if len(values) == 0:
        return float("nan"), float("nan"), float("nan")
    mean = float(np.mean(values))
    if len(values) == 1:
        return mean, mean, mean
    sem = float(np.std(values, ddof=1) / np.sqrt(len(values)))
    return mean, mean - 1.96 * sem, mean + 1.96 * sem


def aggregate(rows):
    grouped = defaultdict(list)
    for row in rows:
        key = (
            row.get("scenario"),
            row.get("condition"),
            row.get("controller_mode"),
            row.get("requested_delay_ms"),
        )
        grouped[key].append(row)

    metrics = [
        "task_duration_s", "task_completed", "minimum_pairwise_distance_m", "collision_threshold_episodes",
        "detection_rate_pct", "path_length_m", "distance_rmse_m", "heading_mae_deg",
        "control_effort_abs_pct_s", "state_transitions", "angular_sign_changes",
        "targets_reached", "ack_rate_pct", "rtt_mean_ms",
        "actual_delay_mean_ms", "actual_delay_p95_ms",
        "vision_processing_mean_ms", "frame_wait_mean_ms", "homography_valid_rate_pct",
    ]
    output = []
    for key, group in sorted(grouped.items(), key=lambda item: str(item[0])):
        row = {
            "scenario": key[0],
            "condition": key[1],
            "controller_mode": key[2],
            "requested_delay_ms": key[3],
            "n": len(group),
        }
        for metric in metrics:
            values = [as_number(item.get(metric)) for item in group]
            finite = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
            mean, low, high = confidence_interval(values)
            row[f"{metric}_mean"] = mean
            row[f"{metric}_std"] = float(np.std(finite, ddof=1)) if len(finite) > 1 else 0.0 if len(finite) == 1 else float("nan")
            row[f"{metric}_ci95_low"] = low
            row[f"{metric}_ci95_high"] = high
        output.append(row)
    return output


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def collect_localization(run_dirs):
    rows = []
    for run_dir in run_dirs:
        validation_path = run_dir / "processed" / "localization_validation.csv"
        manifest_path = run_dir / "manifest.json"
        if not validation_path.exists() or not manifest_path.exists():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        with validation_path.open("r", newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                row.update({
                    "scenario": manifest.get("scenario"),
                    "condition": manifest.get("condition"),
                    "replicate": manifest.get("replicate"),
                    "calibration_enabled": manifest.get("camera_calibration_enabled"),
                    "homography_mode": manifest.get("homography_mode"),
                    "parallax_enabled": manifest.get("parallax_enabled"),
                    "pose_filter_enabled": manifest.get("pose_filter_enabled"),
                })
                rows.append(row)
    return rows


def plot_localization(rows, output_dir):
    if not rows:
        return
    groups = defaultdict(list)
    for row in rows:
        label = f"{row.get('condition')}\n{row.get('variant')}"
        value = as_number(row.get("position_rmse_m"))
        if np.isfinite(value):
            groups[label].append(value * 100.0)
    if not groups:
        return
    labels = sorted(groups)
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    ax.boxplot([groups[label] for label in labels], tick_labels=labels)
    ax.set_ylabel("RMSE de posición (cm)")
    ax.set_title("Comparación de localización")
    ax.tick_params(axis="x", labelrotation=25)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "localization_comparison.png", dpi=180)
    plt.close(fig)


def plot_controller_comparison(rows, output_dir):
    modes_by_scenario = defaultdict(set)
    for row in rows:
        if row.get("controller_mode"):
            modes_by_scenario[row.get("scenario")].add(row.get("controller_mode"))
    comparable_scenarios = {
        scenario for scenario, modes in modes_by_scenario.items() if len(modes) >= 2
    }
    grouped = defaultdict(list)
    for row in rows:
        if row.get("scenario") in comparable_scenarios:
            grouped[row.get("controller_mode")].append(row)
    modes = [mode for mode in sorted(grouped) if mode]
    if len(modes) < 2:
        return
    metrics = [
        ("minimum_pairwise_distance_m", "Distancia mínima (m)"),
        ("task_completed", "Tasa de exito (%)"),
        ("angular_sign_changes", "Cambios de giro"),
        ("control_effort_abs_pct_s", "Esfuerzo (% s)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (metric, label) in zip(axes.flat, metrics):
        if metric == "task_completed":
            rates = [
                100.0 * np.mean([as_number(row.get(metric)) for row in grouped[mode]])
                for mode in modes
            ]
            ax.bar(modes, rates)
            ax.set_ylim(0.0, 105.0)
            ax.set_ylabel(label)
            ax.grid(True, axis="y", alpha=0.3)
            continue
        data = [
            [as_number(row.get(metric)) for row in grouped[mode] if np.isfinite(as_number(row.get(metric)))]
            for mode in modes
        ]
        ax.boxplot(data, tick_labels=modes)
        ax.set_ylabel(label)
        ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "controller_comparison.png", dpi=180)
    plt.close(fig)


def plot_latency(rows, output_dir):
    groups = defaultdict(list)
    for row in rows:
        if row.get("scenario") != "latencia":
            continue
        delay = as_number(row.get("requested_delay_ms"))
        if np.isfinite(delay):
            groups[delay].append(row)
    if len(groups) < 2:
        return
    delays = sorted(groups)
    metrics = [
        ("actual_delay_mean_ms", "Retardo real medio (ms)"),
        ("task_duration_s", "Duracion de tarea (s)"),
        ("heading_mae_deg", "Error angular medio (deg)"),
        ("control_effort_abs_pct_s", "Esfuerzo de control (% s)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (metric, label) in zip(axes.flat, metrics):
        means, lows, highs = [], [], []
        for delay in delays:
            mean, low, high = confidence_interval([as_number(row.get(metric)) for row in groups[delay]])
            means.append(mean)
            lows.append(mean - low)
            highs.append(high - mean)
        ax.errorbar(delays, means, yerr=[lows, highs], marker="o", capsize=3)
        ax.set_xlabel("Retardo inyectado (ms)")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "latency_sweep.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Compara multiples corridas del paper")
    parser.add_argument("--root", type=Path, default=Path("paper/resultados/raw"))
    parser.add_argument("--output", type=Path, default=Path("paper/resultados/procesados"))
    parser.add_argument(
        "--selection",
        type=Path,
        help="JSON con la lista exacta de directorios de corrida que se deben incluir",
    )
    parser.add_argument("--reprocess", action="store_true")
    parser.add_argument(
        "--conditions",
        nargs="+",
        help="Incluye solamente estas etiquetas de condicion",
    )
    parser.add_argument(
        "--completed-only",
        action="store_true",
        help="Excluye corridas abortadas o incompletas",
    )
    parser.add_argument(
        "--valid-only",
        action="store_true",
        help="Incluye exitos y fallos de tarea, pero excluye abortos de infraestructura",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rows = []
    selected_run_dirs = []
    run_dirs = load_selected_runs(args.selection) if args.selection else discover_runs(args.root)
    for run_dir in run_dirs:
        summary_path = run_dir / "processed" / "summary.json"
        if args.reprocess or not summary_path.exists():
            summary = summarize_run(run_dir)
        else:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if args.conditions and str(summary.get("condition")) not in args.conditions:
            continue
        if args.completed_only and not summary.get("task_completed"):
            continue
        if args.valid_only and not summary.get("trial_valid", True):
            continue
        rows.append(flatten_summary(summary))
        selected_run_dirs.append(run_dir)

    write_csv(args.output / "all_runs.csv", rows)
    grouped = aggregate(rows)
    write_csv(args.output / "group_statistics.csv", grouped)
    plot_controller_comparison(rows, args.output)
    plot_latency(rows, args.output)
    localization_rows = collect_localization(selected_run_dirs)
    write_csv(args.output / "localization_all.csv", localization_rows)
    plot_localization(localization_rows, args.output)
    print(f"Corridas encontradas: {len(run_dirs)}")
    print(f"Corridas incluidas: {len(rows)}")
    print(f"Salida: {args.output.resolve()}")


if __name__ == "__main__":
    main()
