import argparse
import json
import select
import socket
import statistics
import time
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1]
CACHE_FILE = PC_DIR / "robots_cache.json"
DISCOVERY_PORT = 37030


def load_endpoints(robot_ids):
    data = json.loads(CACHE_FILE.read_text(encoding="utf-8"))
    cached = data.get("robots", {})
    endpoints = {}
    for robot_id in robot_ids:
        item = cached.get(str(robot_id))
        if item:
            endpoints[robot_id] = (str(item["ip"]), int(item.get("port", 44444)))
    return endpoints


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * percentage)))
    return ordered[index]


def run_diagnostic(
    endpoints,
    duration_s=10.0,
    rate_hz=12.0,
    redundancy=1,
    discovery_bind_port=0,
    ack_probe_interval_s=0.75,
):
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_socket.bind(("", 0))
    command_socket.setblocking(False)
    discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    discovery_socket.bind(("", int(discovery_bind_port)))
    discovery_socket.setblocking(False)

    sequence = 900000
    sent = {robot_id: 0 for robot_id in endpoints}
    probes_sent = {robot_id: 0 for robot_id in endpoints}
    last_probe_t = {robot_id: float("-inf") for robot_id in endpoints}
    pending = {}
    ack_sequences = {robot_id: set() for robot_id in endpoints}
    rtts = {robot_id: [] for robot_id in endpoints}
    discovery_responses = {robot_id: 0 for robot_id in endpoints}
    receive_times = {robot_id: [] for robot_id in endpoints}
    started = time.monotonic()
    next_command = started
    next_discovery = started
    deadline = started + float(duration_s)
    period = 1.0 / max(float(rate_hz), 0.1)

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_command:
                for robot_id, endpoint in endpoints.items():
                    sequence += 1
                    request_ack = (
                        ack_probe_interval_s > 0.0
                        and now - last_probe_t[robot_id] >= ack_probe_interval_s
                    )
                    if request_ack:
                        payload = f"M 0 0 S {sequence}".encode()
                        pending[sequence] = (robot_id, time.perf_counter())
                        probes_sent[robot_id] += 1
                        last_probe_t[robot_id] = now
                    else:
                        payload = b"M 0 0"
                    for _ in range(max(1, int(redundancy))):
                        command_socket.sendto(payload, endpoint)
                    sent[robot_id] += 1
                next_command += period
            if now >= next_discovery:
                for endpoint in endpoints.values():
                    discovery_socket.sendto(
                        b"DISCOVER_ROBOTS", (endpoint[0], DISCOVERY_PORT)
                    )
                next_discovery += 1.0

            readable, _, _ = select.select(
                [command_socket, discovery_socket], [], [], 0.01
            )
            for current_socket in readable:
                data, _ = current_socket.recvfrom(256)
                message = data.decode(errors="ignore").strip()
                values = {}
                for token in message.split()[1:]:
                    if "=" in token:
                        key, value = token.split("=", 1)
                        values[key] = value
                if message.startswith("ACK "):
                    try:
                        robot_id = int(values["ID"])
                        ack_sequence = int(values["SEQ"])
                    except (KeyError, ValueError):
                        continue
                    previous = pending.pop(ack_sequence, None)
                    if previous and ack_sequence not in ack_sequences.get(robot_id, set()):
                        ack_sequences[robot_id].add(ack_sequence)
                        receive_times[robot_id].append(time.monotonic() - started)
                        rtts[robot_id].append(
                            (time.perf_counter() - previous[1]) * 1000.0
                        )
                elif message.startswith("ROBOT_HERE"):
                    try:
                        robot_id = int(values["ID"])
                    except (KeyError, ValueError):
                        continue
                    if robot_id in discovery_responses:
                        discovery_responses[robot_id] += 1
                        receive_times[robot_id].append(time.monotonic() - started)
    finally:
        for _ in range(8):
            for endpoint in endpoints.values():
                command_socket.sendto(b"M 0 0", endpoint)
            time.sleep(0.03)
        command_socket.close()
        discovery_socket.close()

    results = []
    for robot_id, endpoint in endpoints.items():
        robot_rtts = rtts[robot_id]
        timeline = [0.0, *sorted(receive_times[robot_id]), float(duration_s)]
        max_silence_s = max(
            (current - previous for previous, current in zip(timeline, timeline[1:])),
            default=float(duration_s),
        )
        results.append({
            "robot_id": robot_id,
            "endpoint": f"{endpoint[0]}:{endpoint[1]}",
            "commands_sent": sent[robot_id],
            "ack_probes_sent": probes_sent[robot_id],
            "unique_acks": len(ack_sequences[robot_id]),
            "ack_rate_pct": (
                100.0 * len(ack_sequences[robot_id]) / max(probes_sent[robot_id], 1)
            ),
            "discovery_responses": discovery_responses[robot_id],
            "max_rx_silence_s": max_silence_s,
            "rtt_mean_ms": statistics.fmean(robot_rtts) if robot_rtts else None,
            "rtt_p95_ms": percentile(robot_rtts, 0.95),
        })
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Mide UDP con comandos STOP; no mueve los motores."
    )
    parser.add_argument("--robots", nargs="+", type=int, default=[2, 3])
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=12.0)
    parser.add_argument("--redundancy", type=int, default=1)
    parser.add_argument("--discovery-bind-port", type=int, default=0)
    parser.add_argument("--ack-probe-interval", type=float, default=0.0)
    args = parser.parse_args()
    endpoints = load_endpoints(args.robots)
    missing = sorted(set(args.robots) - set(endpoints))
    if missing:
        raise SystemExit(f"Sin endpoint cacheado para: {missing}")
    report = {
        "duration_s": args.duration,
        "rate_hz": args.rate,
        "redundancy": args.redundancy,
        "discovery_bind_port": args.discovery_bind_port,
        "ack_probe_interval_s": args.ack_probe_interval,
        "robots": run_diagnostic(
            endpoints,
            duration_s=args.duration,
            rate_hz=args.rate,
            redundancy=args.redundancy,
            discovery_bind_port=args.discovery_bind_port,
            ack_probe_interval_s=max(0.0, args.ack_probe_interval),
        ),
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
