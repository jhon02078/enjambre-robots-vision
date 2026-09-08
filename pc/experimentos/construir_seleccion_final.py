import argparse
import json
from collections import Counter
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROGRESS = REPO_ROOT / "paper" / "resultados" / "campaign_progress.jsonl"
DEFAULT_OUTPUT = REPO_ROOT / "paper" / "resultados" / "seleccion_final_80.json"

LEGACY_RUN_NAMES = [
    "cruce_2/20260817_205636_cruce_2_proposed_final_r01",
    "cruce_2/20260817_205750_cruce_2_baseline_final_r01",
    "cruce_2/20260817_205848_cruce_2_baseline_final_r02",
    "cruce_2/20260817_205952_cruce_2_proposed_final_r02",
    "cruce_2_perpendicular/20260817_212626_cruce_2_perpendicular_proposed_final_r01",
    "cruce_2_perpendicular/20260817_213232_cruce_2_perpendicular_baseline_final_r01",
    "cruce_2_perpendicular/20260817_213324_cruce_2_perpendicular_baseline_final_r02",
    "cruce_2_perpendicular/20260817_213502_cruce_2_perpendicular_proposed_final_r02",
    "cruce_2_perpendicular/20260817_213650_cruce_2_perpendicular_proposed_final_r03",
    "cruce_2_perpendicular/20260817_213808_cruce_2_perpendicular_baseline_final_r03",
    "cruce_2_perpendicular/20260817_213917_cruce_2_perpendicular_baseline_final_r04",
    "cruce_2_perpendicular/20260817_214023_cruce_2_perpendicular_proposed_final_r04",
    "cruce_2_perpendicular/20260817_214157_cruce_2_perpendicular_proposed_final_r05",
    "cruce_2_perpendicular/20260817_214327_cruce_2_perpendicular_baseline_final_r05",
    "cruce_2_perpendicular/20260817_214429_cruce_2_perpendicular_baseline_final_r06",
    "cruce_2_perpendicular/20260817_214523_cruce_2_perpendicular_proposed_final_r06",
    "cruce_2_perpendicular/20260817_214806_cruce_2_perpendicular_proposed_final_r07",
    "cruce_2_perpendicular/20260817_214924_cruce_2_perpendicular_baseline_final_r07",
    "cruce_2_perpendicular/20260817_215022_cruce_2_perpendicular_baseline_final_r08",
    "cruce_2_perpendicular/20260817_215115_cruce_2_perpendicular_proposed_final_r08",
    "cruce_2_perpendicular/20260817_215246_cruce_2_perpendicular_proposed_final_r09",
    "cruce_2_perpendicular/20260817_215350_cruce_2_perpendicular_baseline_final_r09",
    "cruce_2_perpendicular/20260817_215449_cruce_2_perpendicular_baseline_final_r10",
    "cruce_2_perpendicular/20260818_003529_cruce_2_perpendicular_proposed_final_r10",
]

EXPECTED_GROUPS = {
    ("cruce_2", "baseline_final"): 10,
    ("cruce_2", "proposed_final"): 10,
    ("cruce_2_perpendicular", "baseline_final"): 10,
    ("cruce_2_perpendicular", "proposed_final"): 10,
    ("latencia", "delay_000"): 8,
    ("latencia", "delay_050"): 8,
    ("latencia", "delay_100"): 8,
    ("latencia", "delay_150"): 8,
    ("latencia", "delay_200"): 8,
}


def load_progress(path):
    run_dirs = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        if record.get("trial_valid"):
            run_dirs.append(Path(record["run_dir"]).resolve())
    return run_dirs


def relative_run_path(run_dir):
    return run_dir.relative_to(REPO_ROOT).as_posix()


def validate_runs(run_dirs):
    if len(run_dirs) != 80 or len(set(run_dirs)) != 80:
        raise RuntimeError(f"Se esperaban 80 corridas unicas y se obtuvieron {len(set(run_dirs))}")
    groups = Counter()
    keys = set()
    for run_dir in run_dirs:
        manifest_path = run_dir / "manifest.json"
        summary_path = run_dir / "processed" / "summary.json"
        if not manifest_path.exists() or not summary_path.exists():
            raise FileNotFoundError(f"Corrida incompleta: {run_dir}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if not summary.get("trial_valid"):
            raise RuntimeError(f"Corrida de infraestructura incluida: {run_dir.name}")
        group = (str(manifest.get("scenario")), str(manifest.get("condition")))
        groups[group] += 1
        key = (*group, int(manifest.get("replicate", 0)))
        if key in keys:
            raise RuntimeError(f"Escenario/condicion/repeticion duplicado: {key}")
        keys.add(key)
    if dict(groups) != EXPECTED_GROUPS:
        raise RuntimeError(f"Distribucion inesperada: {dict(groups)}")
    return groups


def main():
    parser = argparse.ArgumentParser(description="Construye la seleccion auditable de 80 corridas")
    parser.add_argument("--progress", type=Path, default=DEFAULT_PROGRESS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    raw_root = REPO_ROOT / "paper" / "resultados" / "raw"
    legacy = [(raw_root / value).resolve() for value in LEGACY_RUN_NAMES]
    run_dirs = legacy + load_progress(args.progress)
    groups = validate_runs(run_dirs)
    payload = {
        "version": 1,
        "description": "Conjunto definitivo y auditable de la campana reducida del paper",
        "total_runs": len(run_dirs),
        "groups": {f"{scenario}|{condition}": count for (scenario, condition), count in groups.items()},
        "excluded_replacements": [
            "paper/resultados/raw/cruce_2_perpendicular/20260817_215544_cruce_2_perpendicular_proposed_final_r10"
        ],
        "runs": [relative_run_path(run_dir) for run_dir in run_dirs],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Seleccion verificada: {len(run_dirs)} corridas")
    print(f"Salida: {args.output.resolve()}")


if __name__ == "__main__":
    main()
