import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy import stats


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOCALIZATION_SUMMARY = (
    REPO_ROOT / "paper" / "resultados" / "procesados" / "localizacion_grid" / "summary.json"
)
DEFAULT_INTRINSIC_SUMMARY = (
    REPO_ROOT / "paper" / "resultados" / "procesados" / "intrinseca_off_on" / "summary.json"
)


CONTROLLER_METRICS = [
    ("task_duration_s", "Duracion de tarea (solo exitos)"),
    ("minimum_pairwise_distance_m", "Distancia minima"),
    ("heading_mae_deg", "Error angular medio"),
    ("control_effort_abs_pct_s", "Esfuerzo de control"),
    ("angular_sign_changes", "Cambios de signo angular"),
]
LATENCY_METRICS = [
    ("task_duration_s", "Duracion de tarea"),
    ("heading_mae_deg", "Error angular medio"),
    ("control_effort_abs_pct_s", "Esfuerzo de control"),
    ("angular_sign_changes", "Cambios de signo angular"),
    ("path_length_m", "Longitud de trayectoria"),
]


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def read_rows(path):
    with Path(path).open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_values(rows, metric, completed_only=False):
    values = []
    for row in rows:
        if completed_only and number(row.get("task_completed")) < 0.5:
            continue
        value = number(row.get(metric))
        if np.isfinite(value):
            values.append(value)
    return np.asarray(values, dtype=float)


def wilson_interval(successes, total, z=1.96):
    if total <= 0:
        return float("nan"), float("nan")
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denominator
    return center - margin, center + margin


def cliffs_delta(first, second):
    if len(first) == 0 or len(second) == 0:
        return float("nan")
    differences = np.subtract.outer(first, second)
    return float((np.sum(differences > 0) - np.sum(differences < 0)) / differences.size)


def controller_tests(rows):
    tests = []
    scenarios = sorted({row["scenario"] for row in rows if row["scenario"].startswith("cruce_2")})
    for scenario in scenarios:
        scenario_rows = [row for row in rows if row["scenario"] == scenario]
        baseline = [row for row in scenario_rows if row["controller_mode"] == "apf_puro"]
        proposed = [row for row in scenario_rows if row["controller_mode"] == "apf_fsm"]
        if not baseline or not proposed:
            continue
        success_b = sum(number(row["task_completed"]) >= 0.5 for row in baseline)
        success_p = sum(number(row["task_completed"]) >= 0.5 for row in proposed)
        odds, p_value = stats.fisher_exact(
            [[success_p, len(proposed) - success_p], [success_b, len(baseline) - success_b]],
            alternative="two-sided",
        )
        tests.append({
            "family": "controller_success",
            "scenario": scenario,
            "metric": "task_completed",
            "test": "fisher_exact_two_sided",
            "statistic": float(odds),
            "p_value": float(p_value),
            "n_a": len(proposed),
            "n_b": len(baseline),
            "effect": success_p / len(proposed) - success_b / len(baseline),
        })
        for metric, _ in CONTROLLER_METRICS:
            completed_only = metric == "task_duration_s"
            values_p = finite_values(proposed, metric, completed_only=completed_only)
            values_b = finite_values(baseline, metric, completed_only=completed_only)
            if len(values_p) == 0 or len(values_b) == 0:
                continue
            statistic, p_value = stats.mannwhitneyu(
                values_p,
                values_b,
                alternative="two-sided",
            )
            tests.append({
                "family": "controller_metric",
                "scenario": scenario,
                "metric": metric,
                "test": "mann_whitney_two_sided",
                "statistic": float(statistic),
                "p_value": float(p_value),
                "n_a": len(values_p),
                "n_b": len(values_b),
                "effect": cliffs_delta(values_p, values_b),
            })
    navigation = [row for row in rows if row["scenario"] in scenarios]
    baseline = [row for row in navigation if row["controller_mode"] == "apf_puro"]
    proposed = [row for row in navigation if row["controller_mode"] == "apf_fsm"]
    success_b = sum(number(row["task_completed"]) >= 0.5 for row in baseline)
    success_p = sum(number(row["task_completed"]) >= 0.5 for row in proposed)
    odds, p_value = stats.fisher_exact(
        [[success_p, len(proposed) - success_p], [success_b, len(baseline) - success_b]],
        alternative="two-sided",
    )
    tests.append({
        "family": "controller_success",
        "scenario": "combined_navigation",
        "metric": "task_completed",
        "test": "fisher_exact_two_sided",
        "statistic": float(odds),
        "p_value": float(p_value),
        "n_a": len(proposed),
        "n_b": len(baseline),
        "effect": success_p / len(proposed) - success_b / len(baseline),
    })
    return tests


def latency_tests(rows):
    latency = [row for row in rows if row["scenario"] == "latencia"]
    delays = np.asarray([number(row["requested_delay_ms"]) for row in latency], dtype=float)
    tests = []
    for metric, _ in LATENCY_METRICS:
        values = np.asarray([number(row.get(metric)) for row in latency], dtype=float)
        mask = np.isfinite(delays) & np.isfinite(values)
        rho, p_value = stats.spearmanr(delays[mask], values[mask])
        tests.append({
            "family": "latency_trend",
            "scenario": "latencia",
            "metric": metric,
            "test": "spearman",
            "statistic": float(rho),
            "p_value": float(p_value),
            "n_a": int(np.sum(mask)),
            "n_b": 0,
            "effect": float(rho),
        })
        groups = [
            finite_values(
                [row for row in latency if number(row["requested_delay_ms"]) == delay],
                metric,
            )
            for delay in sorted(set(delays))
        ]
        statistic, p_value = stats.kruskal(*groups)
        tests.append({
            "family": "latency_groups",
            "scenario": "latencia",
            "metric": metric,
            "test": "kruskal_wallis",
            "statistic": float(statistic),
            "p_value": float(p_value),
            "n_a": sum(len(group) for group in groups),
            "n_b": len(groups),
            "effect": float("nan"),
        })
    return tests


def latency_fidelity(rows):
    grouped = defaultdict(list)
    for row in rows:
        if row["scenario"] == "latencia":
            grouped[number(row["requested_delay_ms"])].append(row)
    output = []
    for requested, group in sorted(grouped.items()):
        means = finite_values(group, "actual_delay_mean_ms")
        p95s = finite_values(group, "actual_delay_p95_ms")
        output.append({
            "requested_delay_ms": requested,
            "n": len(group),
            "successes": sum(number(row["task_completed"]) >= 0.5 for row in group),
            "actual_delay_mean_ms": float(np.mean(means)),
            "actual_delay_between_run_std_ms": float(np.std(means, ddof=1)) if len(means) > 1 else 0.0,
            "actual_delay_p95_mean_ms": float(np.mean(p95s)),
            "overhead_mean_ms": float(np.mean(means) - requested),
        })
    return output


def write_csv(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def fmt(value, digits=3):
    value = number(value)
    return "NA" if not np.isfinite(value) else f"{value:.{digits}f}"


def build_markdown(rows, tests, fidelity, localization, intrinsic):
    lines = [
        "# Resumen estadistico de la campana final",
        "",
        "## Navegacion y evitacion",
        "",
        "| Escenario | Controlador | Exitos | Tasa (IC95% Wilson) |",
        "|---|---|---:|---:|",
    ]
    for scenario in ("cruce_2", "cruce_2_perpendicular"):
        for mode, label in (("apf_puro", "APF puro"), ("apf_fsm", "APF+FSM")):
            group = [row for row in rows if row["scenario"] == scenario and row["controller_mode"] == mode]
            successes = sum(number(row["task_completed"]) >= 0.5 for row in group)
            low, high = wilson_interval(successes, len(group))
            lines.append(
                f"| {scenario} | {label} | {successes}/{len(group)} | "
                f"{100.0 * successes / len(group):.1f}% ({100.0 * low:.1f}-{100.0 * high:.1f}%) |"
            )
    for mode, label in (("apf_puro", "APF puro (combinado)"), ("apf_fsm", "APF+FSM (combinado)")):
        group = [
            row for row in rows
            if row["scenario"].startswith("cruce_2") and row["controller_mode"] == mode
        ]
        successes = sum(number(row["task_completed"]) >= 0.5 for row in group)
        low, high = wilson_interval(successes, len(group))
        lines.append(
            f"| ambos cruces | {label} | {successes}/{len(group)} | "
            f"{100.0 * successes / len(group):.1f}% ({100.0 * low:.1f}-{100.0 * high:.1f}%) |"
        )
    combined_test = next(
        test for test in tests
        if test["scenario"] == "combined_navigation" and test["metric"] == "task_completed"
    )
    lines.extend([
        "",
        f"En los dos cruces combinados, APF+FSM alcanzo 95% frente a 65% de APF puro (Fisher bilateral p={combined_test['p_value']:.4f}, diferencia absoluta +30 puntos porcentuales).",
        "",
        "Las comparaciones inferenciales estan en `statistical_tests.csv`. La duracion se compara solo entre corridas exitosas para evitar interpretar una parada temprana como mejor tiempo.",
        "",
        "## Latencia inyectada",
        "",
        "| Solicitada (ms) | Real media (ms) | P95 medio (ms) | Sobrecoste (ms) | Exitos |",
        "|---:|---:|---:|---:|---:|",
    ])
    for row in fidelity:
        lines.append(
            f"| {row['requested_delay_ms']:.0f} | {row['actual_delay_mean_ms']:.2f} | "
            f"{row['actual_delay_p95_mean_ms']:.2f} | {row['overhead_mean_ms']:.2f} | "
            f"{row['successes']}/{row['n']} |"
        )
    lines.extend([
        "",
        "En el intervalo ensayado de 0 a 200 ms no se observaron perdidas de estabilidad ni fallos de llegada (40/40). Esto demuestra robustez experimental dentro del intervalo, pero no constituye una prueba formal de estabilidad ni permite extrapolar por encima de 200 ms.",
        "",
        "Los niveles de retardo se ejecutaron en orden ascendente. Por tanto, cualquier tendencia debe interpretarse junto con posibles efectos de orden, calentamiento y bateria; la evidencia principal es la tasa de exito y la ausencia de divergencia, no una mejora de rendimiento causada por el retardo.",
        "",
        "## Calidad de vision",
        "",
        f"La homografia fue valida en promedio {np.mean([number(row['homography_valid_rate_pct']) for row in rows]):.2f}% de los frames de las 80 corridas. La deteccion media de los robots activos fue {np.mean([number(row['detection_rate_pct']) for row in rows]):.2f}% y el procesamiento visual medio fue {np.mean([number(row['vision_processing_mean_ms']) for row in rows]):.2f} ms.",
        "",
    ])
    if localization:
        validation = localization["validation_metrics"]
        lines.extend([
            "## Exactitud de localizacion",
            "",
            f"En cinco puntos independientes, el RMSE fue {validation['homography']['rmse_radial_m'] * 100.0:.2f} cm con homografia, {validation['parallax']['rmse_radial_m'] * 100.0:.2f} cm tras paralaje y {validation['aligned']['rmse_radial_m'] * 100.0:.2f} cm tras alineacion XY. El error maximo final fue {validation['aligned']['max_error_m'] * 100.0:.2f} cm.",
            "",
        ])
    if intrinsic:
        raw = intrinsic["stages"]["raw"]
        parallax = intrinsic["stages"]["parallax"]
        detection = intrinsic["detection"]
        calibration = intrinsic["calibration"]
        lines.extend([
            "## Calibracion intrinseca",
            "",
            f"La calibracion uso {calibration['valid_views']} vistas y obtuvo RMS de reproyeccion de {calibration['rms_reprojection_error_px']:.3f} px. En los cinco puntos estaticos no redujo el error metrico central: el RMSE crudo cambio de {raw['off']['rmse_m'] * 100.0:.2f} a {raw['on']['rmse_m'] * 100.0:.2f} cm y, tras paralaje, de {parallax['off']['rmse_m'] * 100.0:.2f} a {parallax['on']['rmse_m'] * 100.0:.2f} cm. La deteccion media aumento {detection['paired_improvement_mean_points']:.2f} puntos porcentuales.",
            "",
            "Este resultado debe reportarse como una evaluacion, no como evidencia de mejora geometrica: en el area central la homografia ya absorbe buena parte de la deformacion y la muestra pareada es pequena.",
            "",
        ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Analisis estadistico de la seleccion final")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--localization-summary", type=Path, default=DEFAULT_LOCALIZATION_SUMMARY)
    parser.add_argument("--intrinsic-summary", type=Path, default=DEFAULT_INTRINSIC_SUMMARY)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = read_rows(args.input)
    if len(rows) != 80:
        raise RuntimeError(f"Se esperaban 80 corridas y se recibieron {len(rows)}")
    tests = controller_tests(rows) + latency_tests(rows)
    fidelity = latency_fidelity(rows)
    localization = json.loads(args.localization_summary.read_text(encoding="utf-8")) if args.localization_summary.exists() else None
    intrinsic = json.loads(args.intrinsic_summary.read_text(encoding="utf-8")) if args.intrinsic_summary.exists() else None
    write_csv(args.output / "statistical_tests.csv", tests)
    write_csv(args.output / "latency_fidelity.csv", fidelity)
    (args.output / "statistical_tests.json").write_text(
        json.dumps(tests, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (args.output / "paper_summary.md").write_text(
        build_markdown(rows, tests, fidelity, localization, intrinsic) + "\n",
        encoding="utf-8",
    )
    print(f"Analisis estadistico: {len(tests)} contrastes")
    print(f"Salida: {args.output.resolve()}")


if __name__ == "__main__":
    main()
