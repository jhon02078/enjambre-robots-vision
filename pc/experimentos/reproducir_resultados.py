import argparse
import filecmp
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
PUBLISHED = REPO_ROOT / "paper" / "resultados"
PUBLIC_CONFIG = REPO_ROOT / "paper" / "configuracion_publica"


def run_script(name, *args):
    command = [sys.executable, str(SCRIPT_DIR / name), *map(str, args)]
    subprocess.run(command, cwd=REPO_ROOT, check=True)


def compare_outputs(generated_dir):
    expected_dir = PUBLISHED / "final_80"
    names = (
        "all_runs.csv",
        "group_statistics.csv",
        "latency_fidelity.csv",
        "paper_summary.md",
        "statistical_tests.csv",
        "statistical_tests.json",
    )
    return [name for name in names if not filecmp.cmp(expected_dir / name, generated_dir / name, shallow=False)]


def main():
    parser = argparse.ArgumentParser(description="Regenera los resultados cuantitativos publicados")
    parser.add_argument(
        "--output",
        type=Path,
        default=PUBLISHED / "regenerados",
        help="Directorio de salida; se conserva el conjunto publicado sin cambios",
    )
    args = parser.parse_args()
    output = args.output.resolve()
    published_final = (PUBLISHED / "final_80").resolve()
    if output == published_final or published_final in output.parents:
        raise ValueError("La salida no puede sobrescribir paper/resultados/final_80")

    localization_output = output / "localizacion_grid"
    intrinsic_output = output / "intrinseca_off_on"
    final_output = output / "final_80"
    output.mkdir(parents=True, exist_ok=True)

    run_script(
        "analizar_localizacion_grid.py",
        "--calibration", PUBLIC_CONFIG / "parallax_measurements.csv",
        "--validation", PUBLIC_CONFIG / "localization_validation.csv",
        "--alignment", PUBLIC_CONFIG / "localization_alignment.json",
        "--output", localization_output,
    )
    run_script(
        "analizar_intrinseca.py",
        "--input", PUBLIC_CONFIG / "intrinsic_validation.csv",
        "--calibration", PUBLIC_CONFIG / "camera_calibration.json",
        "--output", intrinsic_output,
    )
    run_script(
        "comparar_corridas.py",
        "--selection", PUBLISHED / "seleccion_final_80.json",
        "--output", final_output,
        "--reprocess",
        "--valid-only",
    )
    run_script(
        "analisis_estadistico.py",
        "--input", final_output / "all_runs.csv",
        "--output", final_output,
        "--localization-summary", localization_output / "summary.json",
        "--intrinsic-summary", intrinsic_output / "summary.json",
    )

    differences = compare_outputs(final_output)
    if differences:
        print("Regeneración terminada, pero difieren estos archivos:")
        for name in differences:
            print(f"- {name}")
        return 1
    print("Regeneración OK: tablas y resúmenes coinciden con la publicación.")
    print(f"Figuras regeneradas en: {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
