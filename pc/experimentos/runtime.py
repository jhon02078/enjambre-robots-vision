import csv
import heapq
import json
import platform
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


FRAME_FIELDS = [
    "wall_time_iso", "t_wall", "t_perf", "session_elapsed_s", "frame_seq",
    "frame_received_perf", "frame_wait_ms", "processing_ms", "robot_id",
    "detected", "accepted", "workspace_markers_seen", "homography_valid",
    "homography_age_ms", "calibration_enabled", "homography_mode",
    "parallax_enabled", "alignment_enabled", "pose_filter_enabled", "pixel_x", "pixel_y",
    "floor_x", "floor_y", "corrected_x", "corrected_y", "aligned_x", "aligned_y", "raw_yaw",
    "x", "y", "yaw", "pose_age_ms", "cam_x", "cam_y", "cam_z",
]

CONTROL_FIELDS = [
    "wall_time_iso", "t_wall", "t_perf", "session_elapsed_s", "cycle_id",
    "robot_id", "state", "previous_state", "x", "y", "yaw", "pose_age_ms",
    "waypoint_x", "waypoint_y", "final_goal_x", "final_goal_y",
    "distance_error", "desired_heading", "angle_error", "u_att_x", "u_att_y",
    "u_rep_x", "u_rep_y", "u_res_x", "u_res_y", "repulsion_norm",
    "align_factor", "linear_cmd", "angular_cmd", "left_cmd", "right_cmd",
    "controller_mode", "avoid_enabled", "requested_delay_ms", "jitter_ms",
]

NETWORK_FIELDS = [
    "wall_time_iso", "t_wall", "t_perf", "session_elapsed_s", "stage",
    "robot_id", "seq", "left_cmd", "right_cmd", "requested_delay_ms",
    "actual_delay_ms", "scheduled_perf", "sent_perf", "ack_perf", "rtt_ms",
    "esp_rx_ms", "duplicate", "ack_requested", "success", "error",
]

EVENT_FIELDS = [
    "wall_time_iso", "t_wall", "t_perf", "session_elapsed_s", "event",
    "robot_id", "payload_json",
]

SCHEMAS = {
    "frames": FRAME_FIELDS,
    "control": CONTROL_FIELDS,
    "network": NETWORK_FIELDS,
    "events": EVENT_FIELDS,
}


def _utc_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _safe_name(value):
    text = str(value or "sin_nombre").strip().lower()
    cleaned = []
    for char in text:
        if char.isalnum() or char in "-_":
            cleaned.append(char)
        elif char.isspace():
            cleaned.append("_")
    return "".join(cleaned).strip("_") or "sin_nombre"


def _git_metadata(repo_root):
    result = {"commit": None, "branch": None, "dirty": None}
    commands = {
        "commit": ["git", "rev-parse", "HEAD"],
        "branch": ["git", "branch", "--show-current"],
        "dirty": ["git", "status", "--porcelain"],
    }
    for key, command in commands.items():
        try:
            proc = subprocess.run(
                command,
                cwd=str(repo_root),
                capture_output=True,
                text=True,
                check=True,
                timeout=3,
            )
            value = proc.stdout.strip()
            result[key] = bool(value) if key == "dirty" else value
        except (OSError, subprocess.SubprocessError):
            pass
    return result


class ExperimentRecorder:
    """Escritor CSV asincrono para no bloquear vision ni control."""

    def __init__(self, output_root, repo_root=None, queue_size=30000):
        self.output_root = Path(output_root)
        self.repo_root = Path(repo_root) if repo_root else self.output_root
        self.queue = queue.Queue(maxsize=max(int(queue_size), 1000))
        self.active = False
        self.session_dir = None
        self.started_perf = None
        self.started_wall = None
        self.metadata = {}
        self.dropped = {name: 0 for name in SCHEMAS}
        self.counts = {name: 0 for name in SCHEMAS}
        self._thread = None
        self._lock = threading.Lock()

    def start(self, metadata):
        with self._lock:
            if self.active:
                raise RuntimeError("Ya existe una sesion experimental activa")

            scenario = _safe_name(metadata.get("scenario", "custom"))
            condition = _safe_name(metadata.get("condition", "base"))
            replicate = int(metadata.get("replicate", 1))
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_name = f"{stamp}_{scenario}_{condition}_r{replicate:02d}"
            self.session_dir = self.output_root / scenario / run_name
            self.session_dir.mkdir(parents=True, exist_ok=False)

            self.started_perf = time.perf_counter()
            self.started_wall = time.time()
            self.metadata = dict(metadata)
            self.metadata.update({
                "schema_version": 1,
                "run_name": run_name,
                "started_utc": _utc_iso(),
                "python": sys.version,
                "platform": platform.platform(),
                "git": _git_metadata(self.repo_root),
            })
            self.dropped = {name: 0 for name in SCHEMAS}
            self.counts = {name: 0 for name in SCHEMAS}
            self.active = True
            self._write_manifest(status="running")
            self._thread = threading.Thread(target=self._writer_loop, daemon=True)
            self._thread.start()

        self.event("session_start", payload=self.metadata)
        return self.session_dir

    def _write_manifest(self, status, stop_reason=None):
        if self.session_dir is None:
            return
        data = dict(self.metadata)
        data.update({
            "status": status,
            "counts": dict(self.counts),
            "dropped_rows": dict(self.dropped),
        })
        if self.started_perf is not None:
            data["duration_s"] = max(0.0, time.perf_counter() - self.started_perf)
        if stop_reason is not None:
            data["stop_reason"] = str(stop_reason)
            data["finished_utc"] = _utc_iso()
        target = self.session_dir / "manifest.json"
        tmp = target.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        tmp.replace(target)

    def _base_row(self):
        now_perf = time.perf_counter()
        return {
            "wall_time_iso": _utc_iso(),
            "t_wall": time.time(),
            "t_perf": now_perf,
            "session_elapsed_s": (
                now_perf - self.started_perf if self.started_perf is not None else 0.0
            ),
        }

    def record(self, stream, row):
        if not self.active or stream not in SCHEMAS:
            return False
        item = self._base_row()
        item.update(row)
        try:
            self.queue.put_nowait((stream, item))
            return True
        except queue.Full:
            self.dropped[stream] += 1
            return False

    def frame(self, **row):
        return self.record("frames", row)

    def control(self, **row):
        return self.record("control", row)

    def network(self, **row):
        return self.record("network", row)

    def event(self, event, robot_id="", payload=None):
        payload = {} if payload is None else payload
        return self.record("events", {
            "event": str(event),
            "robot_id": robot_id,
            "payload_json": json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        })

    def stop(self, reason="completed"):
        with self._lock:
            if not self.active:
                return self.session_dir
            self.event("session_stop", payload={"reason": reason})
            self.active = False
            self.queue.put(None)
            thread = self._thread

        if thread is not None:
            thread.join(timeout=8.0)
        self._write_manifest(status="completed", stop_reason=reason)
        return self.session_dir

    def _writer_loop(self):
        files = {}
        writers = {}
        try:
            for stream, fields in SCHEMAS.items():
                handle = (self.session_dir / f"{stream}.csv").open(
                    "w", newline="", encoding="utf-8"
                )
                files[stream] = handle
                writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                writers[stream] = writer

            pending_flush = 0
            while True:
                item = self.queue.get()
                if item is None:
                    break
                stream, row = item
                filtered = {field: row.get(field, "") for field in SCHEMAS[stream]}
                writers[stream].writerow(filtered)
                self.counts[stream] += 1
                pending_flush += 1
                if pending_flush >= 100:
                    for handle in files.values():
                        handle.flush()
                    pending_flush = 0
        finally:
            for handle in files.values():
                handle.flush()
                handle.close()


class DelayedCommandDispatcher:
    """Cola monotona para inyectar retardo sin detener el hilo de control."""

    def __init__(self, callback):
        self.callback = callback
        self._heap = []
        self._order = 0
        self._closed = False
        self._cv = threading.Condition()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def schedule(self, due_perf, item):
        with self._cv:
            self._order += 1
            heapq.heappush(self._heap, (float(due_perf), self._order, item))
            self._cv.notify_all()

    def clear(self, robot_id=None):
        with self._cv:
            if robot_id is None:
                self._heap.clear()
            else:
                self._heap = [entry for entry in self._heap if entry[2].get("robot_id") != robot_id]
                heapq.heapify(self._heap)
            self._cv.notify_all()

    def close(self):
        with self._cv:
            self._closed = True
            self._heap.clear()
            self._cv.notify_all()
        self._thread.join(timeout=2.0)

    def _run(self):
        while True:
            with self._cv:
                while not self._closed and not self._heap:
                    self._cv.wait()
                if self._closed:
                    return
                due_perf, _, item = self._heap[0]
                wait_s = due_perf - time.perf_counter()
                if wait_s > 0:
                    self._cv.wait(timeout=wait_s)
                    continue
                heapq.heappop(self._heap)

            actual_delay_ms = max(0.0, (time.perf_counter() - item["scheduled_perf"]) * 1000.0)
            self.callback(item, actual_delay_ms)
