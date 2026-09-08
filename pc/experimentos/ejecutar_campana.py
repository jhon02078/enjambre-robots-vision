import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PC_DIR.parent
SERVER_SCRIPT = PC_DIR / "pc_servidor_vision.py"
ANALYSIS_DIR = REPO_ROOT / "paper" / "resultados" / "batch_logs"
PROGRESS_PATH = REPO_ROOT / "paper" / "resultados" / "campaign_progress.jsonl"
NETWORK_CACHE = PC_DIR / "robots_cache.json"
RUN_DIR_PATTERN = re.compile(r"run_dir=(.+)$")

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from analizar_experimento import summarize_run


def run_key(specification):
    return "|".join(
        str(specification[field])
        for field in ("scenario", "condition", "replicate")
    )


def frontal_specs():
    specifications = []
    for odd_rep in range(3, 10, 2):
        even_rep = odd_rep + 1
        specifications.extend([
            {
                "scenario": "cruce_2", "controller": "apf_fsm",
                "condition": "proposed_final", "replicate": odd_rep,
                "direction": "reverse", "delay_ms": 0.0,
            },
            {
                "scenario": "cruce_2", "controller": "apf_puro",
                "condition": "baseline_final", "replicate": odd_rep,
                "direction": "forward", "delay_ms": 0.0,
            },
            {
                "scenario": "cruce_2", "controller": "apf_puro",
                "condition": "baseline_final", "replicate": even_rep,
                "direction": "reverse", "delay_ms": 0.0,
            },
            {
                "scenario": "cruce_2", "controller": "apf_fsm",
                "condition": "proposed_final", "replicate": even_rep,
                "direction": "forward", "delay_ms": 0.0,
            },
        ])
    return specifications


def latency_specs():
    specifications = []
    for delay_ms in (0, 50, 100, 150, 200):
        for replicate in range(1, 9):
            specifications.append({
                "scenario": "latencia",
                "controller": "apf_fsm",
                "condition": f"delay_{delay_ms:03d}",
                "replicate": replicate,
                "direction": "forward" if replicate % 2 else "reverse",
                "delay_ms": float(delay_ms),
            })
    return specifications


def load_completed_keys():
    completed = set()
    if not PROGRESS_PATH.exists():
        return completed
    for line in PROGRESS_PATH.read_text(encoding="utf-8").splitlines():
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if record.get("trial_valid"):
            completed.add(record.get("key"))
    return completed


def append_progress(record):
    PROGRESS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with PROGRESS_PATH.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def robot_endpoints():
    if not NETWORK_CACHE.exists():
        return []
    try:
        payload = json.loads(NETWORK_CACHE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    endpoints = []
    for robot_id, info in payload.get("robots", {}).items():
        if int(robot_id) not in {2, 3} or not info.get("ip"):
            continue
        endpoints.append((str(info["ip"]), int(info.get("port", 44444))))
    return endpoints


def send_stop(repetitions=10):
    endpoints = robot_endpoints()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _ in range(int(repetitions)):
            for endpoint in endpoints:
                sock.sendto(b"M 0 0", endpoint)
                time.sleep(0.025)
    finally:
        sock.close()
    print(f"[BATCH] STOP x{repetitions} endpoints={len(endpoints)}", flush=True)


def hidden_process_options():
    if os.name != "nt":
        return {}
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startupinfo.wShowWindow = 0
    return {
        "startupinfo": startupinfo,
        "creationflags": subprocess.CREATE_NO_WINDOW,
    }


def execute_one(specification, index, total, scenario_timeout_s):
    key = run_key(specification)
    slug = key.replace("|", "_")
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    log_path = ANALYSIS_DIR / f"{slug}.log"
    command = [
        sys.executable,
        str(SERVER_SCRIPT),
        "--run-paper-scenario", specification["scenario"],
        "--scenario-direction", specification["direction"],
        "--controller-mode", specification["controller"],
        "--condition", specification["condition"],
        "--replicate", str(specification["replicate"]),
        "--scenario-timeout", str(float(scenario_timeout_s)),
        "--safety-distance", "0.12",
        "--start-accuracy", "0.05",
        "--network-abort-timeout", "120",
        "--delay-ms", str(specification.get("delay_ms", 0.0)),
        "--jitter-ms", str(specification.get("jitter_ms", 0.0)),
    ]
    print(
        f"[BATCH] {index}/{total} START {key} dir={specification['direction']} "
        f"delay={specification.get('delay_ms', 0.0):.0f}ms",
        flush=True,
    )
    run_dir = None
    with log_path.open("w", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            **hidden_process_options(),
        )
        for line in process.stdout:
            log_handle.write(line)
            log_handle.flush()
            print(line.rstrip(), flush=True)
            match = RUN_DIR_PATTERN.search(line.strip())
            if match and match.group(1) != "-":
                candidate = Path(match.group(1).strip())
                if candidate.exists():
                    run_dir = candidate
        return_code = process.wait()
    send_stop()
    if return_code != 0 or run_dir is None:
        print(
            f"[BATCH] STOP campaign: return_code={return_code} run_dir={run_dir}",
            flush=True,
        )
        return None
    summary = summarize_run(run_dir)
    record = {
        "key": key,
        "run_dir": str(run_dir),
        "trial_valid": bool(summary.get("trial_valid")),
        "outcome": summary.get("outcome"),
        "stop_reason": summary.get("stop_reason"),
        "task_completed": bool(summary.get("task_completed")),
        "task_duration_s": summary.get("task_duration_s"),
        "minimum_pairwise_distance_m": summary.get("minimum_pairwise_distance_m"),
        "timestamp": time.time(),
    }
    append_progress(record)
    print("[BATCH] RESULT " + json.dumps(record, ensure_ascii=False), flush=True)
    if not record["trial_valid"]:
        return None
    if record["outcome"] == "task_failure":
        paths = [float(robot.get("path_length_m", 0.0)) for robot in summary.get("robots", [])]
        if paths and min(paths) < 0.10:
            print(
                "[BATCH] STOP campaign: movimiento menor a 0.10m; revisar hardware",
                flush=True,
            )
            return None
    return summary


def main():
    parser = argparse.ArgumentParser(description="Ejecuta bloques fisicos reanudables")
    parser.add_argument("--phase", choices=("frontal", "latency", "all"), required=True)
    parser.add_argument("--cooldown-every", type=int, default=4)
    parser.add_argument("--cooldown-seconds", type=float, default=45.0)
    parser.add_argument("--scenario-timeout", type=float, default=60.0)
    args = parser.parse_args()
    specifications = []
    if args.phase in {"frontal", "all"}:
        specifications.extend(frontal_specs())
    if args.phase in {"latency", "all"}:
        specifications.extend(latency_specs())
    completed = load_completed_keys()
    pending = [specification for specification in specifications if run_key(specification) not in completed]
    print(
        f"[BATCH] phase={args.phase} planned={len(specifications)} pending={len(pending)}",
        flush=True,
    )
    for index, specification in enumerate(pending, start=1):
        summary = execute_one(specification, index, len(pending), args.scenario_timeout)
        if summary is None:
            send_stop()
            raise SystemExit(2)
        if (
            args.cooldown_every > 0
            and index < len(pending)
            and index % args.cooldown_every == 0
        ):
            print(f"[BATCH] cooldown {args.cooldown_seconds:.0f}s", flush=True)
            time.sleep(max(0.0, args.cooldown_seconds))
    send_stop()
    print(f"[BATCH] COMPLETE phase={args.phase} runs={len(pending)}", flush=True)


if __name__ == "__main__":
    main()
