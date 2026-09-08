import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]


def portable_path(path):
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


VALID_TASK_FAILURE_PREFIXES = (
    "scenario_stalled",
    "scenario_timeout",
    "critical_separation",
)


def classify_trial_outcome(stop_reason, task_completed):
    """Separa fallos del controlador de abortos que invalidan la corrida."""
    reason = str(stop_reason or "").strip()
    if task_completed or reason == "completed":
        return True, "success"
    if any(reason.startswith(prefix) for prefix in VALID_TASK_FAILURE_PREFIXES):
        return True, "task_failure"
    return False, "infrastructure_abort"


NUMERIC_FIELDS = {
    "t_wall", "t_perf", "session_elapsed_s", "frame_seq", "frame_received_perf",
    "frame_wait_ms", "processing_ms", "robot_id", "detected", "accepted",
    "workspace_markers_seen", "homography_valid", "homography_age_ms", "pixel_x",
    "pixel_y", "floor_x", "floor_y", "corrected_x", "corrected_y",
    "aligned_x", "aligned_y", "raw_yaw",
    "x", "y", "yaw", "pose_age_ms", "cam_x", "cam_y", "cam_z", "cycle_id",
    "waypoint_x", "waypoint_y", "final_goal_x", "final_goal_y", "distance_error",
    "desired_heading", "angle_error", "u_att_x", "u_att_y", "u_rep_x", "u_rep_y",
    "u_res_x", "u_res_y", "repulsion_norm", "align_factor", "linear_cmd",
    "angular_cmd", "left_cmd", "right_cmd", "requested_delay_ms", "jitter_ms",
    "seq", "actual_delay_ms", "scheduled_perf", "sent_perf", "ack_perf", "rtt_ms",
    "esp_rx_ms", "duplicate", "ack_requested", "success",
}


def as_float(value, default=float("nan")):
    try:
        if value in (None, ""):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def ack_requested(row):
    value = row.get("ack_requested")
    if value in (None, "") or not np.isfinite(as_float(value)):
        return True
    return bool(as_float(value, 0.0))


def load_csv(path):
    path = Path(path)
    if not path.exists():
        return []
    rows = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            for key in NUMERIC_FIELDS.intersection(row):
                row[key] = as_float(row[key])
            rows.append(row)
    return rows


def load_run(run_dir):
    run_dir = Path(run_dir)
    manifest_path = run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    events = load_csv(run_dir / "events.csv")
    for row in events:
        try:
            row["payload"] = json.loads(row.get("payload_json") or "{}")
        except json.JSONDecodeError:
            row["payload"] = {}
    return {
        "run_dir": run_dir,
        "manifest": manifest,
        "frames": load_csv(run_dir / "frames.csv"),
        "control": load_csv(run_dir / "control.csv"),
        "network": load_csv(run_dir / "network.csv"),
        "events": events,
    }


def path_length(rows):
    total = 0.0
    previous = None
    for row in sorted(rows, key=lambda item: item["t_perf"]):
        if not row.get("detected") or not row.get("accepted"):
            continue
        point = (row.get("x"), row.get("y"))
        if not all(np.isfinite(point)):
            continue
        if previous is not None:
            step = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if step <= 0.30:
                total += step
        previous = point
    return total


def integrate_control_effort(rows):
    rows = sorted(rows, key=lambda item: item["t_perf"])
    effort_abs = 0.0
    effort_sq = 0.0
    for previous, current in zip(rows, rows[1:]):
        dt = min(max(current["t_perf"] - previous["t_perf"], 0.0), 0.5)
        left = as_float(previous.get("left_cmd"), 0.0)
        right = as_float(previous.get("right_cmd"), 0.0)
        effort_abs += (abs(left) + abs(right)) * dt
        effort_sq += (left * left + right * right) * dt
    return effort_abs, effort_sq


def state_statistics(rows):
    rows = sorted(rows, key=lambda item: item["t_perf"])
    dwell = defaultdict(float)
    transitions = 0
    previous_state = None
    angular_sign_changes = 0
    previous_sign = 0
    for previous, current in zip(rows, rows[1:]):
        state = previous.get("state") or "UNKNOWN"
        dt = min(max(current["t_perf"] - previous["t_perf"], 0.0), 0.5)
        dwell[state] += dt
        if previous_state is not None and state != previous_state:
            transitions += 1
        previous_state = state
        angular = as_float(previous.get("angular_cmd"), 0.0)
        sign = 1 if angular > 1.0 else -1 if angular < -1.0 else 0
        if sign and previous_sign and sign != previous_sign:
            angular_sign_changes += 1
        if sign:
            previous_sign = sign
    return dict(dwell), transitions, angular_sign_changes


def pairwise_metrics(frames, threshold_m):
    grouped = defaultdict(dict)
    for row in frames:
        if row.get("detected") and row.get("accepted"):
            grouped[int(row["frame_seq"])][int(row["robot_id"])] = row

    samples = defaultdict(list)
    for poses in grouped.values():
        ids = sorted(poses)
        for i, rid_a in enumerate(ids):
            for rid_b in ids[i + 1:]:
                a, b = poses[rid_a], poses[rid_b]
                d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                t = max(a["session_elapsed_s"], b["session_elapsed_s"])
                samples[(rid_a, rid_b)].append((t, d))

    minimum = min((distance for values in samples.values() for _, distance in values), default=float("nan"))
    episodes = 0
    for values in samples.values():
        active = False
        for _, distance in sorted(values):
            below = distance < threshold_m
            if below and not active:
                episodes += 1
            active = below
    return minimum, episodes, samples


def target_successes(events):
    successes = defaultdict(int)
    for row in events:
        if row.get("event") == "target_reached" and row.get("robot_id") not in (None, ""):
            successes[int(float(row["robot_id"]))] += 1
    return dict(successes)


def scenario_robot_ids(events):
    for row in reversed(events):
        if row.get("event") != "scenario_started":
            continue
        robots = row.get("payload", {}).get("robots", [])
        if robots:
            return [int(value) for value in robots]
    return []


def task_timing(events):
    starts = [row for row in events if row.get("event") == "scenario_started"]
    if not starts:
        return float("nan"), False, 0
    start = starts[-1]
    expected = {int(value) for value in start.get("payload", {}).get("robots", [])}
    reached_rows = [
        row for row in events
        if row.get("event") == "target_reached" and row["t_perf"] >= start["t_perf"]
    ]
    reached = {int(float(row["robot_id"])) for row in reached_rows if row.get("robot_id") not in (None, "")}
    completed = bool(expected) and expected.issubset(reached)
    if not reached_rows:
        return float("nan"), completed, len(reached)
    end_perf = max(row["t_perf"] for row in reached_rows)
    return max(0.0, end_perf - start["t_perf"]), completed, len(reached)


def localization_validation(bundle, output_dir):
    starts = [event for event in bundle["events"] if event.get("event") == "ground_truth_start"]
    results = []
    for start in starts:
        payload = start.get("payload", {})
        rid = int(payload["robot_id"])
        duration = float(payload.get("duration_s", 5.0))
        t0 = start["t_perf"]
        rows = [
            row for row in bundle["frames"]
            if int(row["robot_id"]) == rid
            and row.get("detected")
            and t0 <= row["t_perf"] <= t0 + duration
        ]
        if not rows:
            continue
        truth_x = float(payload["x"])
        truth_y = float(payload["y"])
        truth_yaw = math.radians(float(payload.get("yaw_deg", 0.0)))
        for variant, x_key, y_key, yaw_key in [
            ("homography", "floor_x", "floor_y", "raw_yaw"),
            ("parallax", "corrected_x", "corrected_y", "raw_yaw"),
            ("aligned", "aligned_x", "aligned_y", "raw_yaw"),
            ("filtered", "x", "y", "yaw"),
        ]:
            xs = np.asarray([row[x_key] for row in rows if np.isfinite(row.get(x_key, np.nan))])
            ys = np.asarray([row[y_key] for row in rows if np.isfinite(row.get(y_key, np.nan))])
            yaws = np.asarray([row[yaw_key] for row in rows if np.isfinite(row.get(yaw_key, np.nan))])
            count = min(len(xs), len(ys))
            if count == 0:
                continue
            errors = np.hypot(xs[:count] - truth_x, ys[:count] - truth_y)
            yaw_errors = np.arctan2(np.sin(yaws - truth_yaw), np.cos(yaws - truth_yaw)) if len(yaws) else np.array([])
            results.append({
                "label": payload.get("label", ""),
                "robot_id": rid,
                "variant": variant,
                "truth_x": truth_x,
                "truth_y": truth_y,
                "truth_yaw_deg": math.degrees(truth_yaw),
                "samples": count,
                "mean_x": float(np.mean(xs)),
                "mean_y": float(np.mean(ys)),
                "position_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
                "position_mean_m": float(np.mean(errors)),
                "position_p95_m": float(np.percentile(errors, 95)),
                "position_max_m": float(np.max(errors)),
                "jitter_x_std_m": float(np.std(xs)),
                "jitter_y_std_m": float(np.std(ys)),
                "yaw_mae_deg": float(np.mean(np.abs(np.degrees(yaw_errors)))) if len(yaw_errors) else float("nan"),
                "yaw_std_deg": float(np.std(np.degrees(yaw_errors))) if len(yaw_errors) else float("nan"),
            })

    if not results:
        return []

    csv_path = output_dir / "localization_validation.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)

    variants = sorted({item["variant"] for item in results})
    data = [[item["position_rmse_m"] * 100.0 for item in results if item["variant"] == variant] for variant in variants]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.boxplot(data, tick_labels=variants)
    ax.set_ylabel("RMSE de posición (cm)")
    ax.set_title("Validación de localización")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "localization_validation.png", dpi=180)
    plt.close(fig)
    return results


def summarize_run(run_dir, output_dir=None):
    bundle = load_run(run_dir)
    manifest = bundle["manifest"]
    output_dir = Path(output_dir) if output_dir else Path(run_dir) / "processed"
    output_dir.mkdir(parents=True, exist_ok=True)
    collision_threshold = float(manifest.get("collision_threshold_m", 0.12))

    frame_by_robot = defaultdict(list)
    for row in bundle["frames"]:
        frame_by_robot[int(row["robot_id"])].append(row)
    control_by_robot = defaultdict(list)
    for row in bundle["control"]:
        control_by_robot[int(row["robot_id"])].append(row)

    successes = target_successes(bundle["events"])
    active_robot_ids = scenario_robot_ids(bundle["events"])
    if not active_robot_ids:
        active_robot_ids = sorted(control_by_robot)
    if not active_robot_ids:
        active_robot_ids = sorted(
            rid
            for rid, rows in frame_by_robot.items()
            if any(row.get("detected") and row.get("accepted") for row in rows)
        )
    robot_metrics = []
    for rid in active_robot_ids:
        frames = frame_by_robot[rid]
        controls = control_by_robot[rid]
        detected = sum(1 for row in frames if row.get("detected"))
        accepted = sum(1 for row in frames if row.get("accepted"))
        effort_abs, effort_sq = integrate_control_effort(controls)
        dwell, transitions, oscillations = state_statistics(controls)
        distance_errors = [row["distance_error"] for row in controls if np.isfinite(row.get("distance_error", np.nan))]
        angle_errors = [abs(row["angle_error"]) for row in controls if np.isfinite(row.get("angle_error", np.nan))]
        robot_metrics.append({
            "robot_id": rid,
            "frames": len(frames),
            "detection_rate_pct": 100.0 * detected / max(len(frames), 1),
            "accepted_rate_pct": 100.0 * accepted / max(len(frames), 1),
            "path_length_m": path_length(frames),
            "distance_rmse_m": float(np.sqrt(np.mean(np.square(distance_errors)))) if distance_errors else float("nan"),
            "heading_mae_deg": float(np.degrees(np.mean(angle_errors))) if angle_errors else float("nan"),
            "control_effort_abs_pct_s": effort_abs,
            "control_effort_sq_pct2_s": effort_sq,
            "state_transitions": transitions,
            "angular_sign_changes": oscillations,
            "target_reached_events": successes.get(rid, 0),
            "state_dwell_s": dwell,
        })

    active_frames = [
        row for row in bundle["frames"]
        if int(row["robot_id"]) in active_robot_ids
    ]
    min_distance, collision_episodes, pair_samples = pairwise_metrics(
        active_frames, collision_threshold
    )
    active_network = [
        row for row in bundle["network"]
        if int(row["robot_id"]) in active_robot_ids
    ]
    ack_rows = [
        row for row in active_network
        if row.get("stage") == "ack"
        and row.get("duplicate") != 1
        and np.isfinite(row.get("rtt_ms", np.nan))
    ]
    rtts = np.asarray([row["rtt_ms"] for row in ack_rows], dtype=float)
    requested_delay_ms = float(manifest.get("requested_delay_ms", 0.0))
    actual_delays = np.asarray(
        [
            row["actual_delay_ms"]
            for row in active_network
            if row.get("stage") == "sent"
            and np.isfinite(row.get("actual_delay_ms", np.nan))
            and abs(row.get("requested_delay_ms", 0.0) - requested_delay_ms) < 1e-6
        ],
        dtype=float,
    )
    sent = sum(1 for row in active_network if row.get("stage") == "sent")
    ack_probes_sent = sum(
        1 for row in active_network
        if row.get("stage") == "sent" and ack_requested(row)
    )
    network_by_robot = []
    for rid in active_robot_ids:
        robot_rows = [row for row in active_network if int(row["robot_id"]) == rid]
        robot_sent = [row for row in robot_rows if row.get("stage") == "sent"]
        robot_probes = [row for row in robot_sent if ack_requested(row)]
        robot_acks = [
            row for row in robot_rows
            if row.get("stage") == "ack"
            and row.get("duplicate") != 1
            and np.isfinite(row.get("rtt_ms", np.nan))
        ]
        robot_rtts = np.asarray([row["rtt_ms"] for row in robot_acks], dtype=float)
        robot_actual_delays = np.asarray(
            [
                row["actual_delay_ms"]
                for row in robot_sent
                if np.isfinite(row.get("actual_delay_ms", np.nan))
                and abs(row.get("requested_delay_ms", 0.0) - requested_delay_ms) < 1e-6
            ],
            dtype=float,
        )
        network_by_robot.append({
            "robot_id": rid,
            "commands_sent": len(robot_sent),
            "ack_probes_sent": len(robot_probes),
            "acks_received": len(robot_acks),
            "ack_rate_pct": 100.0 * len(robot_acks) / max(len(robot_probes), 1),
            "rtt_mean_ms": float(np.mean(robot_rtts)) if len(robot_rtts) else float("nan"),
            "rtt_p95_ms": float(np.percentile(robot_rtts, 95)) if len(robot_rtts) else float("nan"),
            "actual_delay_mean_ms": float(np.mean(robot_actual_delays)) if len(robot_actual_delays) else float("nan"),
            "actual_delay_p95_ms": float(np.percentile(robot_actual_delays, 95)) if len(robot_actual_delays) else float("nan"),
            "last_sent_elapsed_s": max(
                (row["session_elapsed_s"] for row in robot_sent), default=float("nan")
            ),
            "last_ack_elapsed_s": max(
                (row["session_elapsed_s"] for row in robot_acks), default=float("nan")
            ),
        })
    task_duration_s, task_completed, robots_reached = task_timing(bundle["events"])
    unique_frames = {}
    for row in bundle["frames"]:
        unique_frames.setdefault(int(row["frame_seq"]), row)
    processing_values = np.asarray(
        [row["processing_ms"] for row in unique_frames.values() if np.isfinite(row.get("processing_ms", np.nan))],
        dtype=float,
    )
    frame_wait_values = np.asarray(
        [row["frame_wait_ms"] for row in unique_frames.values() if np.isfinite(row.get("frame_wait_ms", np.nan))],
        dtype=float,
    )
    homography_valid_rate = 100.0 * sum(
        1 for row in unique_frames.values() if row.get("homography_valid")
    ) / max(len(unique_frames), 1)

    stop_reason = manifest.get("stop_reason")
    trial_valid, outcome = classify_trial_outcome(stop_reason, task_completed)
    summary = {
        "schema_version": 1,
        "run_dir": portable_path(run_dir),
        "scenario": manifest.get("scenario"),
        "condition": manifest.get("condition"),
        "replicate": manifest.get("replicate"),
        "controller_mode": manifest.get("controller_mode"),
        "scenario_direction": next(
            (
                row.get("payload", {}).get("direction")
                for row in reversed(bundle["events"])
                if row.get("event") == "scenario_started"
            ),
            None,
        ),
        "requested_delay_ms": requested_delay_ms,
        "jitter_ms": manifest.get("jitter_ms", 0.0),
        "duration_s": manifest.get("duration_s"),
        "task_duration_s": task_duration_s,
        "task_completed": task_completed,
        "trial_valid": trial_valid,
        "outcome": outcome,
        "stop_reason": stop_reason,
        "robots_reached": robots_reached,
        "collision_threshold_m": collision_threshold,
        "minimum_pairwise_distance_m": min_distance,
        "collision_threshold_episodes": collision_episodes,
        "commands_sent": sent,
        "ack_probes_sent": ack_probes_sent,
        "acks_received": len(ack_rows),
        "ack_rate_pct": 100.0 * len(ack_rows) / max(ack_probes_sent, 1),
        "rtt_mean_ms": float(np.mean(rtts)) if len(rtts) else float("nan"),
        "rtt_p95_ms": float(np.percentile(rtts, 95)) if len(rtts) else float("nan"),
        "approx_one_way_mean_ms": float(np.mean(rtts) / 2.0) if len(rtts) else float("nan"),
        "actual_delay_mean_ms": float(np.mean(actual_delays)) if len(actual_delays) else float("nan"),
        "actual_delay_std_ms": float(np.std(actual_delays, ddof=1)) if len(actual_delays) > 1 else 0.0 if len(actual_delays) == 1 else float("nan"),
        "actual_delay_p95_ms": float(np.percentile(actual_delays, 95)) if len(actual_delays) else float("nan"),
        "network_by_robot": network_by_robot,
        "vision_processing_mean_ms": float(np.mean(processing_values)) if len(processing_values) else float("nan"),
        "vision_processing_p95_ms": float(np.percentile(processing_values, 95)) if len(processing_values) else float("nan"),
        "frame_wait_mean_ms": float(np.mean(frame_wait_values)) if len(frame_wait_values) else float("nan"),
        "frame_wait_p95_ms": float(np.percentile(frame_wait_values, 95)) if len(frame_wait_values) else float("nan"),
        "homography_valid_rate_pct": homography_valid_rate,
        "robots": robot_metrics,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    if robot_metrics:
        flat_rows = []
        for row in robot_metrics:
            flat = dict(row)
            flat["state_dwell_s"] = json.dumps(flat["state_dwell_s"], separators=(",", ":"))
            flat_rows.append(flat)
        with (output_dir / "summary_by_robot.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(flat_rows[0]))
            writer.writeheader()
            writer.writerows(flat_rows)

    plot_trajectories(bundle, output_dir, collision_threshold)
    plot_timeseries(bundle, output_dir)
    plot_pairwise(pair_samples, output_dir, collision_threshold)
    localization_validation(bundle, output_dir)
    return summary


def plot_trajectories(bundle, output_dir, collision_threshold):
    fig, ax = plt.subplots(figsize=(6, 6))
    by_robot = defaultdict(list)
    for row in bundle["frames"]:
        if row.get("detected") and row.get("accepted"):
            by_robot[int(row["robot_id"])].append(row)
    for rid, rows in sorted(by_robot.items()):
        rows.sort(key=lambda row: row["t_perf"])
        ax.plot([row["x"] for row in rows], [row["y"] for row in rows], label=f"R{rid}")
        if rows:
            ax.scatter(rows[0]["x"], rows[0]["y"], marker="o", s=35)
            ax.scatter(rows[-1]["x"], rows[-1]["y"], marker="x", s=45)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Trayectorias (umbral de contacto: {collision_threshold:.3f} m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "trajectories.png", dpi=180)
    plt.close(fig)


def plot_timeseries(bundle, output_dir):
    by_robot = defaultdict(list)
    for row in bundle["control"]:
        by_robot[int(row["robot_id"])].append(row)
    if not by_robot:
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for rid, rows in sorted(by_robot.items()):
        rows.sort(key=lambda row: row["session_elapsed_s"])
        t = [row["session_elapsed_s"] for row in rows]
        axes[0].plot(t, [row["distance_error"] for row in rows], label=f"R{rid}")
        axes[1].plot(t, [math.degrees(row["angle_error"]) for row in rows], label=f"R{rid}")
        axes[2].plot(t, [row["left_cmd"] for row in rows], label=f"R{rid} L")
        axes[2].plot(t, [row["right_cmd"] for row in rows], linestyle="--", label=f"R{rid} R")
    axes[0].set_ylabel("Error distancia (m)")
    axes[1].set_ylabel("Error angular (deg)")
    axes[2].set_ylabel("Comando (%)")
    axes[2].set_xlabel("Tiempo (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "timeseries.png", dpi=180)
    plt.close(fig)


def plot_pairwise(samples, output_dir, threshold):
    if not samples:
        return
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for pair, values in sorted(samples.items()):
        values.sort()
        ax.plot([item[0] for item in values], [item[1] for item in values], label=f"R{pair[0]}-R{pair[1]}")
    ax.axhline(threshold, color="red", linestyle="--", label="umbral")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Distancia entre centros (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "pairwise_distance.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Procesa una sesion experimental del paper")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    summary = summarize_run(args.run_dir, args.output)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
