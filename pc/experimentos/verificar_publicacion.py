import csv
import hashlib
import json
import re
import sys
from collections import Counter
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SELECTION_PATH = REPO_ROOT / "paper" / "resultados" / "seleccion_final_80.json"
FINAL_RUNS_PATH = REPO_ROOT / "paper" / "resultados" / "final_80" / "all_runs.csv"
HASHES_PATH = REPO_ROOT / "paper" / "reproducibilidad" / "SHA256SUMS.txt"
REQUIRED_RUN_FILES = ("manifest.json", "frames.csv", "control.csv", "network.csv", "events.csv")
PRIVATE_PATH_PATTERN = re.compile(r"(?:[A-Za-z]:[\\/]+Users[\\/]|/home/|paper_pruebas)", re.IGNORECASE)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_hashes(path):
    expected = {}
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        if not line.strip():
            continue
        digest, relative = line.split("  ", 1)
        expected[relative] = digest.lower()
    return expected


def main():
    errors = []
    selection = json.loads(SELECTION_PATH.read_text(encoding="utf-8"))
    run_names = selection.get("runs", [])
    if len(run_names) != selection.get("total_runs") or len(set(run_names)) != len(run_names):
        errors.append("La selección no contiene 80 rutas únicas.")

    groups = Counter()
    dirty_manifests = 0
    for relative in run_names:
        run_dir = (REPO_ROOT / relative).resolve()
        try:
            run_dir.relative_to(REPO_ROOT)
        except ValueError:
            errors.append(f"Ruta fuera del repositorio: {relative}")
            continue
        for name in REQUIRED_RUN_FILES:
            path = run_dir / name
            if not path.is_file():
                errors.append(f"Falta {relative}/{name}")
                continue
            if PRIVATE_PATH_PATTERN.search(path.read_text(encoding="utf-8", errors="ignore")):
                errors.append(f"Ruta privada no saneada: {relative}/{name}")
        manifest_path = run_dir / "manifest.json"
        if not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        groups[f"{manifest.get('scenario')}|{manifest.get('condition')}"] += 1
        dirty_manifests += int(bool(manifest.get("git", {}).get("dirty")))

    expected_groups = Counter(selection.get("groups", {}))
    if groups != expected_groups:
        errors.append(f"Grupos inesperados: {dict(groups)}")

    if not FINAL_RUNS_PATH.is_file():
        errors.append("Falta el resumen final all_runs.csv.")
    else:
        with FINAL_RUNS_PATH.open("r", encoding="utf-8-sig", newline="") as handle:
            final_rows = list(csv.DictReader(handle))
        if len(final_rows) != selection.get("total_runs"):
            errors.append(f"all_runs.csv tiene {len(final_rows)} filas.")
        if any(PRIVATE_PATH_PATTERN.search(row.get("run_dir", "")) for row in final_rows):
            errors.append("all_runs.csv contiene rutas privadas.")

    if not HASHES_PATH.is_file():
        errors.append("Falta SHA256SUMS.txt.")
        hashes = {}
    else:
        hashes = load_hashes(HASHES_PATH)
    for relative, expected in hashes.items():
        path = REPO_ROOT / relative
        if not path.is_file():
            errors.append(f"Falta archivo inventariado: {relative}")
        elif sha256(path) != expected:
            errors.append(f"Hash distinto: {relative}")

    print(f"Corridas publicadas: {len(run_names)}")
    print(f"Grupos verificados: {len(groups)}")
    print(f"Manifiestos con dirty=true preservados: {dirty_manifests}")
    print(f"Archivos con SHA-256 verificado: {len(hashes)}")
    if errors:
        print("VERIFICACIÓN FALLIDA:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("VERIFICACIÓN OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
