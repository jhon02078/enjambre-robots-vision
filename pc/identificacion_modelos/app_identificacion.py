import csv
import json
import math
import socket
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

try:
    from .analysis import (
        estimate_frequency_response,
        fit_transfer_model,
        combined_differential_model,
        pid_recommendations,
        plot_results,
        wrap_pi,
        write_json,
    )
except ImportError:
    from analysis import (
        estimate_frequency_response,
        fit_transfer_model,
        combined_differential_model,
        pid_recommendations,
        plot_results,
        wrap_pi,
        write_json,
    )


ROBOT_IDS = [1, 2, 3, 10]
WORKSPACE_IDS = [4, 5, 6, 7]
DISCOVERY_PORT = 37030
DISCOVERY_QUERY = b"DISCOVER_ROBOTS"
ROBOT_CMD_PORT = 44444
CMD_RATE_HZ = 20
POSE_HISTORY_N = 5
HOMOGRAPHY_HOLD_S = 2.5
ROBOT_POSE_MARGIN_M = 0.08
ROBOT_MAX_JUMP_M = 0.35
ROBOT_RAW_DEADZONE_M = 0.008
ROBOT_SLOW_ALPHA = 0.18
ROBOT_FAST_ALPHA = 0.55
ROBOT_FAST_MOVE_M = 0.08
ROBOT_YAW_DEADZONE_RAD = math.radians(2.0)
RESULTS_DIR = Path(__file__).with_name("resultados")

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_PARAMS.minMarkerPerimeterRate = 0.003
ARUCO_PARAMS.polygonalApproxAccuracyRate = 0.06
try:
    ARUCO_PARAMS.useAruco3Detection = True
    ARUCO_PARAMS.minSideLengthCanonicalImg = 16
    ARUCO_PARAMS.minMarkerDistanceRate = 0.005
except AttributeError:
    pass
ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
ARUCO_PARAMS.cornerRefinementWinSize = 5
ARUCO_PARAMS.cornerRefinementMaxIterations = 50
ARUCO_PARAMS.cornerRefinementMinAccuracy = 0.01
ARUCO_PARAMS.adaptiveThreshWinSizeMin = 5
ARUCO_PARAMS.adaptiveThreshWinSizeMax = 45
ARUCO_PARAMS.adaptiveThreshWinSizeStep = 10
ARUCO_PARAMS.adaptiveThreshConstant = 7
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class IdentificationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Identificacion de modelos por vision")
        self.root.geometry("1280x820")

        self.running = True
        self.abort_experiment = threading.Event()
        self.experiment_running = False
        self.lock = threading.Lock()

        self.cap = None
        self.latest_frame = None
        self.latest_display = None
        self.homography = None
        self.homography_t = 0.0
        self.ws_centers = {}
        self.ws_last_seen = {}
        self.robot_state = {rid: None for rid in ROBOT_IDS}
        self.pose_hist = {rid: deque(maxlen=POSE_HISTORY_N) for rid in ROBOT_IDS}
        self.discovered = {rid: None for rid in ROBOT_IDS}
        self.samples = []
        self.last_output_dir = None

        self.url_camera = tk.StringVar(value="http://192.168.1.10:5000/video")
        self.real_width = tk.DoubleVar(value=1.25)
        self.real_height = tk.DoubleVar(value=1.25)
        self.robot_id = tk.IntVar(value=1)
        self.linear_amplitude_pct = tk.DoubleVar(value=35.0)
        self.angular_amplitude_pct = tk.DoubleVar(value=30.0)
        self.linear_freq_min_hz = tk.DoubleVar(value=0.20)
        self.linear_freq_max_hz = tk.DoubleVar(value=1.20)
        self.linear_freq_points = tk.IntVar(value=12)
        self.angular_freq_min_hz = tk.DoubleVar(value=0.20)
        self.angular_freq_max_hz = tk.DoubleVar(value=2.0)
        self.angular_freq_points = tk.IntVar(value=14)
        self.repeats_per_freq = tk.IntVar(value=2)
        self.analysis_sample_hz = tk.DoubleVar(value=40.0)
        self.derivative_window_s = tk.DoubleVar(value=0.18)
        self.min_quality = tk.DoubleVar(value=0.05)
        self.settle_cycles = tk.DoubleVar(value=1.0)
        self.measure_cycles = tk.DoubleVar(value=2.0)
        self.edge_margin_m = tk.DoubleVar(value=0.10)
        self.max_excursion_m = tk.DoubleVar(value=0.18)
        self.status = tk.StringVar(value="Listo. Conecta la camara y espera deteccion de robots.")

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.disc_sock.bind(("", DISCOVERY_PORT))
        self.disc_sock.settimeout(0.05)

        self._setup_ui()

        self.video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self.video_thread.start()
        self.discovery_thread = threading.Thread(target=self._discovery_loop, daemon=True)
        self.discovery_thread.start()
        self._ui_loop()

    def _setup_ui(self):
        top = tk.Frame(self.root, bg="#e5e5e5", pady=6)
        top.pack(side=tk.TOP, fill=tk.X)

        tk.Label(top, text="Cam URL:", bg="#e5e5e5").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.url_camera, width=36).pack(side=tk.LEFT, padx=3)
        tk.Button(top, text="Conectar", command=self.connect_camera).pack(side=tk.LEFT, padx=4)

        tk.Label(top, text="W:", bg="#e5e5e5").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.real_width, width=6).pack(side=tk.LEFT)
        tk.Label(top, text="H:", bg="#e5e5e5").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.real_height, width=6).pack(side=tk.LEFT)

        tk.Label(top, text="Robot:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(12, 2))
        ttk.Combobox(top, textvariable=self.robot_id, values=ROBOT_IDS, width=4, state="readonly").pack(side=tk.LEFT)

        row_freq = tk.Frame(self.root, bg="#e5e5e5", pady=4)
        row_freq.pack(side=tk.TOP, fill=tk.X)
        tk.Label(row_freq, text="Lineal Amp(%):", bg="#e5e5e5").pack(side=tk.LEFT)
        tk.Entry(row_freq, textvariable=self.linear_amplitude_pct, width=6).pack(side=tk.LEFT)
        tk.Label(row_freq, text="f min/max:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(8, 2))
        tk.Entry(row_freq, textvariable=self.linear_freq_min_hz, width=6).pack(side=tk.LEFT)
        tk.Entry(row_freq, textvariable=self.linear_freq_max_hz, width=6).pack(side=tk.LEFT)
        tk.Label(row_freq, text="N:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(6, 2))
        tk.Entry(row_freq, textvariable=self.linear_freq_points, width=4).pack(side=tk.LEFT)

        tk.Label(row_freq, text=" | Angular Amp(%):", bg="#e5e5e5").pack(side=tk.LEFT, padx=(12, 2))
        tk.Entry(row_freq, textvariable=self.angular_amplitude_pct, width=6).pack(side=tk.LEFT)
        tk.Label(row_freq, text="f min/max:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(8, 2))
        tk.Entry(row_freq, textvariable=self.angular_freq_min_hz, width=6).pack(side=tk.LEFT)
        tk.Entry(row_freq, textvariable=self.angular_freq_max_hz, width=6).pack(side=tk.LEFT)
        tk.Label(row_freq, text="N:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(6, 2))
        tk.Entry(row_freq, textvariable=self.angular_freq_points, width=4).pack(side=tk.LEFT)
        tk.Label(row_freq, text="reps:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row_freq, textvariable=self.repeats_per_freq, width=4).pack(side=tk.LEFT)

        row2 = tk.Frame(self.root, bg="#e5e5e5", pady=4)
        row2.pack(side=tk.TOP, fill=tk.X)
        tk.Label(row2, text="settle cycles:", bg="#e5e5e5").pack(side=tk.LEFT)
        tk.Entry(row2, textvariable=self.settle_cycles, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="measure cycles:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.measure_cycles, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="margen borde(m):", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.edge_margin_m, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="excursion max(m):", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.max_excursion_m, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="analisis Hz:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.analysis_sample_hz, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="vent.deriv(s):", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.derivative_window_s, width=5).pack(side=tk.LEFT)
        tk.Label(row2, text="calidad min:", bg="#e5e5e5").pack(side=tk.LEFT, padx=(10, 2))
        tk.Entry(row2, textvariable=self.min_quality, width=5).pack(side=tk.LEFT)

        tk.Button(row2, text="Barrido lineal", command=lambda: self.start_experiment(["lineal"])).pack(side=tk.LEFT, padx=8)
        tk.Button(row2, text="Barrido angular", command=lambda: self.start_experiment(["angular"])).pack(side=tk.LEFT, padx=4)
        tk.Button(row2, text="Lineal + angular", command=lambda: self.start_experiment(["lineal", "angular"])).pack(side=tk.LEFT, padx=4)
        tk.Button(row2, text="STOP", bg="#d32f2f", fg="white", command=self.stop_experiment).pack(side=tk.LEFT, padx=8)

        self.lbl_status = tk.Label(self.root, textvariable=self.status, anchor="w", bg="#111", fg="#7CFC00", font=("Consolas", 10))
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X)

        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)
        self.panel_video = tk.LabelFrame(main, text="Camara")
        self.panel_video.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.lbl_video = tk.Label(self.panel_video, bg="black")
        self.lbl_video.pack(fill=tk.BOTH, expand=True)

        self.panel_map = tk.LabelFrame(main, text="Mapa / estado")
        self.panel_map.pack(side=tk.RIGHT, fill=tk.BOTH, padx=6, pady=6)
        self.canvas = tk.Canvas(self.panel_map, bg="white", width=430, height=560)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def connect_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.url_camera.get().strip())
        self.status.set("Camara conectada. Esperando homografia y robots...")

    def _camera_subnet_broadcast(self):
        try:
            parsed = urlparse(self.url_camera.get().strip())
            host = parsed.hostname
            if not host:
                return None
            parts = host.split(".")
            if len(parts) != 4:
                return None
            return ".".join(parts[:3] + ["255"])
        except Exception:
            return None

    def _discovery_loop(self):
        while self.running:
            endpoints = [("255.255.255.255", DISCOVERY_PORT)]
            cam_bcast = self._camera_subnet_broadcast()
            if cam_bcast:
                endpoints.append((cam_bcast, DISCOVERY_PORT))

            try:
                for endpoint in endpoints:
                    try:
                        self.disc_sock.sendto(DISCOVERY_QUERY, endpoint)
                    except OSError:
                        pass

                t_end = time.time() + 0.25
                while time.time() < t_end:
                    try:
                        data, addr = self.disc_sock.recvfrom(256)
                    except socket.timeout:
                        continue
                    msg = data.decode(errors="ignore").strip()
                    if not msg.startswith("ROBOT_HERE"):
                        continue
                    rid = None
                    port = ROBOT_CMD_PORT
                    for part in msg.split():
                        if part.startswith("ID="):
                            try:
                                rid = int(part.split("=", 1)[1])
                            except ValueError:
                                rid = None
                        elif part.startswith("CMDPORT="):
                            try:
                                port = int(part.split("=", 1)[1])
                            except ValueError:
                                port = ROBOT_CMD_PORT
                    if rid in ROBOT_IDS:
                        with self.lock:
                            self.discovered[rid] = {"ip": addr[0], "port": port, "t": time.time()}
            except Exception:
                pass
            time.sleep(0.35)

    def send_robot_cmd(self, rid, left_pct, right_pct):
        left_pct = int(clamp(left_pct, -100, 100))
        right_pct = int(clamp(right_pct, -100, 100))
        with self.lock:
            info = self.discovered.get(rid)
        if not info:
            return False
        msg = f"M {left_pct} {right_pct}".encode()
        try:
            self.cmd_sock.sendto(msg, (info["ip"], info["port"]))
            time.sleep(0.003)
            self.cmd_sock.sendto(msg, (info["ip"], info["port"]))
            return True
        except OSError:
            return False

    def stop_robot(self, rid):
        for _ in range(5):
            self.send_robot_cmd(rid, 0, 0)
            time.sleep(0.015)

    def _video_loop(self):
        while self.running:
            if self.cap is None:
                time.sleep(0.05)
                continue
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.03)
                continue
            with self.lock:
                self.latest_frame = frame

    def _marker_center(self, corners_4x2):
        return np.mean(corners_4x2, axis=0)

    def _transform_point(self, H, x, y):
        p = np.array([x, y, 1.0], dtype=np.float64)
        q = H @ p
        if abs(q[2]) < 1e-12:
            return None
        return float(q[0] / q[2]), float(q[1] / q[2])

    def _valid_robot_pose(self, x, y, prev_st, now):
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        margin = ROBOT_POSE_MARGIN_M
        if not (-margin <= x <= W + margin and -margin <= y <= H + margin):
            return False

        if prev_st is None:
            return True

        px, py = prev_st["x"], prev_st["y"]
        prev_in_workspace = (-margin <= px <= W + margin and -margin <= py <= H + margin)
        if not prev_in_workspace:
            return True

        age = now - prev_st.get("t", 0.0)
        jump = math.hypot(x - px, y - py)
        if age < 1.0 and jump > ROBOT_MAX_JUMP_M:
            return False
        return True

    def _filtered_pose(self, rid, x, y, yaw, prev_st, now):
        hist = self.pose_hist[rid]
        if prev_st is not None and (now - prev_st.get("t", 0.0)) > 1.0:
            hist.clear()
        hist.append((x, y, yaw))
        xs = [p[0] for p in hist]
        ys = [p[1] for p in hist]
        yaws = [p[2] for p in hist]
        x_med = float(np.median(xs))
        y_med = float(np.median(ys))
        sin_sum = sum(math.sin(a) for a in yaws)
        cos_sum = sum(math.cos(a) for a in yaws)
        yaw_med = math.atan2(sin_sum, cos_sum)

        if prev_st is None:
            return x_med, y_med, yaw_med

        prev_x, prev_y = prev_st["x"], prev_st["y"]
        prev_yaw = prev_st["yaw"]
        dist_moved = math.hypot(x_med - prev_x, y_med - prev_y)
        yaw_diff = wrap_pi(yaw_med - prev_yaw)

        if dist_moved < ROBOT_RAW_DEADZONE_M:
            x_final = prev_x
            y_final = prev_y
        else:
            move_ratio = clamp(dist_moved / ROBOT_FAST_MOVE_M, 0.0, 1.0)
            alpha = ROBOT_SLOW_ALPHA + (ROBOT_FAST_ALPHA - ROBOT_SLOW_ALPHA) * move_ratio
            x_final = (prev_x * (1.0 - alpha)) + (x_med * alpha)
            y_final = (prev_y * (1.0 - alpha)) + (y_med * alpha)

        if abs(yaw_diff) < ROBOT_YAW_DEADZONE_RAD:
            yaw_final = prev_yaw
        else:
            yaw_alpha = 0.22 if abs(yaw_diff) < math.radians(12.0) else 0.45
            yaw_final = wrap_pi(prev_yaw + yaw_diff * yaw_alpha)

        return x_final, y_final, yaw_final

    def process_frame(self, frame):
        display = frame.copy()
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        frame_sharp = cv2.filter2D(frame, -1, kernel)
        corners, ids, _ = ARUCO_DETECTOR.detectMarkers(frame_sharp)
        now = time.time()
        W = float(self.real_width.get())
        H = float(self.real_height.get())

        if ids is None:
            return display

        ids = ids.flatten().tolist()
        cv2.aruco.drawDetectedMarkers(display, corners, np.array(ids))

        raw_centers = {}
        for i, mid in enumerate(ids):
            raw_centers[mid] = self._marker_center(corners[i][0])

        heavy_lock_threshold_px = 15.0
        ws_alpha = 0.8
        for mid in WORKSPACE_IDS:
            if mid in raw_centers:
                raw = np.array(raw_centers[mid], dtype=np.float32)
                prev = self.ws_centers.get(mid)
                if prev is None:
                    self.ws_centers[mid] = raw
                    self.ws_last_seen[mid] = now
                else:
                    dist_moved = float(np.linalg.norm(raw - prev))
                    if dist_moved > heavy_lock_threshold_px:
                        self.ws_centers[mid] = (ws_alpha * prev) + ((1.0 - ws_alpha) * raw)
                    self.ws_last_seen[mid] = now

        can_draw_poly = all(mid in self.ws_centers for mid in WORKSPACE_IDS)
        if can_draw_poly:
            pts_draw = np.array([self.ws_centers[mid] for mid in WORKSPACE_IDS], dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(display, [pts_draw], True, (0, 255, 255), 3)

        def recent_workspace(mid):
            return mid in self.ws_centers and now - self.ws_last_seen.get(mid, 0.0) <= HOMOGRAPHY_HOLD_S

        if all(recent_workspace(mid) for mid in WORKSPACE_IDS):
            img_pts = np.array([self.ws_centers[4], self.ws_centers[5], self.ws_centers[6], self.ws_centers[7]], dtype=np.float32)
            world_pts = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], dtype=np.float32)
            Hnew = cv2.getPerspectiveTransform(img_pts, world_pts)

            with self.lock:
                hold = None if self.homography is None else self.homography.copy()

            if hold is not None:
                a = Hnew / (Hnew[2, 2] + 1e-9)
                b = hold / (hold[2, 2] + 1e-9)
                jump = float(np.linalg.norm(a - b))
                if jump > 0.8:
                    Hnew = None

            with self.lock:
                if Hnew is not None:
                    if self.homography is None:
                        self.homography = Hnew
                    else:
                        h_alpha = 0.95
                        self.homography = (h_alpha * self.homography) + ((1.0 - h_alpha) * Hnew)
                    self.homography_t = now

        with self.lock:
            Huse = None if self.homography is None else self.homography.copy()

        if Huse is None:
            return display

        for i, mid in enumerate(ids):
            if mid not in ROBOT_IDS:
                continue
            c = corners[i][0]
            center = self._marker_center(c)
            pos = self._transform_point(Huse, float(center[0]), float(center[1]))
            p0 = self._transform_point(Huse, float(c[0][0]), float(c[0][1]))
            p1 = self._transform_point(Huse, float(c[1][0]), float(c[1][1]))
            if pos is None or p0 is None or p1 is None:
                continue
            yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            with self.lock:
                prev_st = self.robot_state[mid]
            if not self._valid_robot_pose(pos[0], pos[1], prev_st, now):
                continue
            x, y, yaw = self._filtered_pose(mid, pos[0], pos[1], yaw, prev_st, now)
            with self.lock:
                self.robot_state[mid] = {"x": x, "y": y, "yaw": yaw, "t": now}
            cv2.putText(display, f"ID:{mid}", (int(center[0]), int(center[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        return display

    def _safe_to_run(self, rid):
        now = time.time()
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        margin = float(self.edge_margin_m.get())
        with self.lock:
            st = self.robot_state.get(rid)
            h_age = now - self.homography_t
        if h_age > HOMOGRAPHY_HOLD_S:
            return False, "homografia no disponible"
        if st is None or now - st.get("t", 0.0) > 0.45:
            return False, f"ArUco robot {rid} perdido"
        if not (margin <= st["x"] <= W - margin and margin <= st["y"] <= H - margin):
            return False, "robot cerca del borde o fuera del area"
        return True, ""

    def _start_pose_ok(self, rid):
        ok, reason = self._safe_to_run(rid)
        if not ok:
            return False, reason

        W = float(self.real_width.get())
        H = float(self.real_height.get())
        margin = max(float(self.edge_margin_m.get()), float(self.max_excursion_m.get()) + 0.04)
        with self.lock:
            st = self.robot_state.get(rid)
        if st is None:
            return False, f"ArUco robot {rid} perdido"
        if not (margin <= st["x"] <= W - margin and margin <= st["y"] <= H - margin):
            return False, (
                "coloca el robot mas cerca del centro: el barrido necesita espacio "
                f"para moverse al menos {float(self.max_excursion_m.get()):.2f} m"
            )
        return True, ""

    def start_experiment(self, modes):
        if self.experiment_running:
            messagebox.showwarning("Experimento activo", "Ya hay un barrido en ejecucion.")
            return
        self.abort_experiment.clear()
        self.experiment_running = True
        self.samples = []
        thread = threading.Thread(target=self._run_experiment, args=(modes,), daemon=True)
        thread.start()

    def stop_experiment(self):
        self.abort_experiment.set()
        self.stop_robot(int(self.robot_id.get()))
        self.status.set("STOP enviado. Experimento cancelado.")

    def _mode_amplitude(self, mode):
        if mode == "lineal":
            return clamp(float(self.linear_amplitude_pct.get()), 5.0, 90.0)
        return clamp(float(self.angular_amplitude_pct.get()), 5.0, 90.0)

    def _frequency_list(self, mode):
        if mode == "lineal":
            fmin = max(float(self.linear_freq_min_hz.get()), 0.01)
            fmax = max(float(self.linear_freq_max_hz.get()), fmin)
            n = max(int(self.linear_freq_points.get()), 2)
        else:
            fmin = max(float(self.angular_freq_min_hz.get()), 0.01)
            fmax = max(float(self.angular_freq_max_hz.get()), fmin)
            n = max(int(self.angular_freq_points.get()), 2)
        return np.geomspace(fmin, fmax, n).tolist()

    def _mode_frequency_list(self, mode):
        freqs = self._frequency_list(mode)
        if mode != "lineal":
            return freqs

        # El barrido lineal a frecuencia muy baja desplaza demasiado al robot.
        # Se filtra con una cota conservadora de velocidad esperada para no
        # gastar la prueba llegando al borde.
        amp = max(self._mode_amplitude(mode), 1.0)
        max_exc = max(float(self.max_excursion_m.get()), 0.05)
        assumed_gain_m_s_pct = 0.006
        min_safe_freq = (assumed_gain_m_s_pct * amp) / (2.0 * math.pi * max_exc)
        safe = [f for f in freqs if f >= min_safe_freq]
        if not safe:
            safe = [max(min_safe_freq, min(freqs))]
        if len(safe) < len(freqs):
            self.status.set(
                f"Lineal: se omitieron frecuencias < {min_safe_freq:.3f} Hz por limite de excursion."
            )
        return safe

    def _append_stop_sample(self, samples, rid, mode, freq, repeat, segment_t):
        with self.lock:
            st = None if self.robot_state[rid] is None else dict(self.robot_state[rid])
        if st is None:
            return
        samples.append({
            "t": time.time(),
            "mode": mode,
            "freq_hz": float(freq),
            "repeat": int(repeat),
            "segment_t": float(segment_t),
            "left_cmd": 0.0,
            "right_cmd": 0.0,
            "x_m": float(st["x"]),
            "y_m": float(st["y"]),
            "yaw_rad": float(st["yaw"]),
            "yaw_unwrapped_rad": float(st["yaw"]),
            "v_m_s": 0.0,
            "w_rad_s": 0.0,
        })

    def _run_experiment(self, modes):
        rid = int(self.robot_id.get())
        settle = max(float(self.settle_cycles.get()), 0.0)
        measure = max(float(self.measure_cycles.get()), 0.5)
        repeats = max(int(self.repeats_per_freq.get()), 1)
        acquisition_sample_hz = max(float(self.analysis_sample_hz.get()), 5.0)
        sample_period_s = 1.0 / acquisition_sample_hz
        all_samples = []
        incomplete = []

        try:
            ok, reason = self._start_pose_ok(rid)
            if not ok:
                self.status.set(f"No se inicia: {reason}")
                return

            pause_experiment = False
            for mode in modes:
                if pause_experiment:
                    break
                for freq in self._mode_frequency_list(mode):
                    if pause_experiment:
                        break
                    for repeat in range(1, repeats + 1):
                        if self.abort_experiment.is_set():
                            return
                        amp = self._mode_amplitude(mode)
                        prev_pose = None
                        last_yaw_unwrapped = None
                        segment_samples = []
                        segment_ok = True
                        total_s = (settle + measure) / freq
                        t0 = time.time()
                        next_cmd = 0.0
                        next_sample = 0.0
                        origin = None
                        self.status.set(
                            f"R{rid} {mode}: {freq:.3f} Hz rep {repeat}/{repeats} durante {total_s:.1f}s"
                        )

                        while time.time() - t0 < total_s:
                            if self.abort_experiment.is_set():
                                return

                            ok, reason = self._safe_to_run(rid)
                            if not ok:
                                self.status.set(f"Auto-stop en {mode} {freq:.3f} Hz: {reason}. Continuando si es seguro.")
                                incomplete.append({
                                    "mode": mode,
                                    "freq_hz": float(freq),
                                    "repeat": int(repeat),
                                    "reason": reason,
                                })
                                segment_ok = False
                                break

                            now = time.time()
                            seg_t = now - t0
                            u = amp * math.sin(2.0 * math.pi * freq * seg_t)
                            if mode == "lineal":
                                left, right = u, u
                            else:
                                left, right = -u, u

                            if now >= next_cmd:
                                self.send_robot_cmd(rid, left, right)
                                next_cmd = now + (1.0 / CMD_RATE_HZ)

                            if now >= next_sample:
                                next_sample = now + sample_period_s
                                with self.lock:
                                    st = None if self.robot_state[rid] is None else dict(self.robot_state[rid])
                                if st is not None:
                                    if origin is None:
                                        origin = (st["x"], st["y"])
                                    if mode == "lineal":
                                        excursion = math.hypot(st["x"] - origin[0], st["y"] - origin[1])
                                        if excursion > float(self.max_excursion_m.get()):
                                            reason = f"excursion lineal {excursion:.2f} m supera limite"
                                            self.status.set(f"Auto-stop en {mode} {freq:.3f} Hz: {reason}.")
                                            incomplete.append({
                                                "mode": mode,
                                                "freq_hz": float(freq),
                                                "repeat": int(repeat),
                                                "reason": reason,
                                            })
                                            segment_ok = False
                                            break

                                    yaw = st["yaw"]
                                    if last_yaw_unwrapped is None:
                                        yaw_unwrapped = yaw
                                    else:
                                        yaw_unwrapped = last_yaw_unwrapped + wrap_pi(yaw - last_yaw_unwrapped)
                                    last_yaw_unwrapped = yaw_unwrapped

                                    v_m_s = 0.0
                                    w_rad_s = 0.0
                                    if prev_pose is not None:
                                        dt_pose = max(now - prev_pose["t"], 1e-3)
                                        dx = st["x"] - prev_pose["x"]
                                        dy = st["y"] - prev_pose["y"]
                                        v_m_s = (dx * math.cos(st["yaw"]) + dy * math.sin(st["yaw"])) / dt_pose
                                        w_rad_s = (yaw_unwrapped - prev_pose["yaw_unwrapped"]) / dt_pose
                                    prev_pose = {"x": st["x"], "y": st["y"], "yaw_unwrapped": yaw_unwrapped, "t": now}

                                    segment_samples.append({
                                        "t": now,
                                        "mode": mode,
                                        "freq_hz": float(freq),
                                        "repeat": int(repeat),
                                        "segment_t": float(seg_t),
                                        "left_cmd": float(left),
                                        "right_cmd": float(right),
                                        "x_m": float(st["x"]),
                                        "y_m": float(st["y"]),
                                        "yaw_rad": float(st["yaw"]),
                                        "yaw_unwrapped_rad": float(yaw_unwrapped),
                                        "v_m_s": float(v_m_s),
                                        "w_rad_s": float(w_rad_s),
                                    })
                            time.sleep(0.002)

                        self.stop_robot(rid)
                        if segment_ok:
                            all_samples.extend(segment_samples)
                        else:
                            self._append_stop_sample(all_samples, rid, mode, freq, repeat, time.time() - t0)
                            time.sleep(0.8)
                            ok, reason = self._safe_to_run(rid)
                            if not ok:
                                self.status.set(f"Experimento pausado: {reason}. Reubica el robot y vuelve a iniciar.")
                                pause_experiment = True
                                break
                        time.sleep(0.8)

            self.samples = all_samples
            self._save_results(rid, modes, settle, incomplete)
        finally:
            self.stop_robot(rid)
            self.experiment_running = False

    def _save_results(self, rid, modes, settle_cycles, incomplete=None):
        if incomplete is None:
            incomplete = []
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = RESULTS_DIR / f"robot_{rid}" / stamp
        out_dir.mkdir(parents=True, exist_ok=True)
        self.last_output_dir = out_dir

        csv_path = out_dir / "samples.csv"
        fieldnames = [
            "t",
            "mode",
            "freq_hz",
            "repeat",
            "segment_t",
            "left_cmd",
            "right_cmd",
            "x_m",
            "y_m",
            "yaw_rad",
            "yaw_unwrapped_rad",
            "v_m_s",
            "w_rad_s",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)

        analysis_config = {
            "linear_amplitude_pct": float(self.linear_amplitude_pct.get()),
            "angular_amplitude_pct": float(self.angular_amplitude_pct.get()),
            "linear_freq_min_hz": float(self.linear_freq_min_hz.get()),
            "linear_freq_max_hz": float(self.linear_freq_max_hz.get()),
            "linear_freq_points": int(self.linear_freq_points.get()),
            "angular_freq_min_hz": float(self.angular_freq_min_hz.get()),
            "angular_freq_max_hz": float(self.angular_freq_max_hz.get()),
            "angular_freq_points": int(self.angular_freq_points.get()),
            "repeats_per_freq": int(self.repeats_per_freq.get()),
            "analysis_sample_hz": float(self.analysis_sample_hz.get()),
            "derivative_window_s": float(self.derivative_window_s.get()),
            "min_quality": float(self.min_quality.get()),
            "settle_cycles": float(self.settle_cycles.get()),
            "measure_cycles": float(self.measure_cycles.get()),
            "edge_margin_m": float(self.edge_margin_m.get()),
            "max_excursion_m": float(self.max_excursion_m.get()),
        }

        freq_response = {}
        models = {}
        quality_metrics = {}
        for mode in modes:
            resp = estimate_frequency_response(
                self.samples,
                mode,
                settle_cycles,
                sample_hz=analysis_config["analysis_sample_hz"],
                derivative_window_s=analysis_config["derivative_window_s"],
                min_quality=analysis_config["min_quality"],
            )
            freq_response[mode] = resp
            quality_metrics[mode] = [
                {
                    "freq_hz": item.get("freq_hz"),
                    "n_repeats": item.get("n_repeats", 0),
                    "coherence_like_mean": item.get("coherence_like_mean"),
                    "coherence_like_min": item.get("coherence_like_min"),
                    "snr_db_mean": item.get("snr_db_mean"),
                    "magnitude_std": item.get("magnitude_std"),
                    "phase_std_deg": item.get("phase_std_deg"),
                    "fit_weight": item.get("fit_weight"),
                    "quality_rejected": item.get("quality_rejected", []),
                }
                for item in resp
            ]
            if len(resp) >= 2:
                models[mode] = fit_transfer_model(resp)

        write_json(out_dir / "frequency_response.json", freq_response)
        write_json(out_dir / "quality_metrics.json", quality_metrics)
        write_json(out_dir / "model.json", models)
        write_json(
            out_dir / "modelo_diferencial.json",
            combined_differential_model(models.get("lineal"), models.get("angular")),
        )
        pid = pid_recommendations(models.get("lineal"), models.get("angular"))
        write_json(out_dir / "pid_params.json", pid)
        write_json(out_dir / "experiment_summary.json", {
            "robot_id": rid,
            "modes_requested": modes,
            "n_samples": len(self.samples),
            "analysis_config": analysis_config,
            "incomplete_frequencies": incomplete,
            "notes": [
                "Modelo unico en coordenadas diferenciales: u_v=(L+R)/2, u_w=(R-L)/2.",
                "Los puntos Bode se remuestrean a tiempo uniforme y se derivan con suavizado antes de ajustar.",
                "Las repeticiones se promedian por frecuencia usando pesos de calidad.",
                "Si faltan frecuencias lineales, fueron omitidas, rechazadas por calidad o detenidas para no llegar al borde.",
            ],
        })
        plot_msg = plot_results(out_dir, freq_response, models, self.samples)

        msg = f"Resultados guardados: {out_dir}"
        if incomplete:
            msg += f" | incompletas: {len(incomplete)}"
        if plot_msg:
            msg += f" | {plot_msg}"
        self.status.set(msg)

    def _draw_map(self):
        self.canvas.delete("all")
        cw = max(self.canvas.winfo_width(), 100)
        ch = max(self.canvas.winfo_height(), 100)
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        margin = 35
        scale = min((cw - 2 * margin) / max(W, 1e-6), (ch - 2 * margin) / max(H, 1e-6))
        ox = (cw - W * scale) * 0.5
        oy = (ch - H * scale) * 0.5

        def wm(x, y):
            return ox + x * scale, oy + (H - y) * scale

        x0, y0 = wm(0, H)
        x1, y1 = wm(W, 0)
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="black", width=2)
        margin_m = float(self.edge_margin_m.get())
        sx0, sy0 = wm(margin_m, H - margin_m)
        sx1, sy1 = wm(W - margin_m, margin_m)
        self.canvas.create_rectangle(sx0, sy0, sx1, sy1, outline="red", dash=(4, 3))

        with self.lock:
            states = {rid: None if self.robot_state[rid] is None else dict(self.robot_state[rid]) for rid in ROBOT_IDS}
            disc = {rid: self.discovered[rid] for rid in ROBOT_IDS}
            h_ok = self.homography is not None and time.time() - self.homography_t < HOMOGRAPHY_HOLD_S

        self.canvas.create_text(8, 12, anchor="w", text=f"Homografia: {'OK' if h_ok else '---'}")
        y_text = 30
        for rid in ROBOT_IDS:
            st = states[rid]
            net = disc[rid]
            net_txt = "---" if not net else f"{net['ip']} age {time.time() - net['t']:.1f}s"
            self.canvas.create_text(8, y_text, anchor="w", text=f"R{rid}: {net_txt}")
            y_text += 18
            if st is None:
                continue
            mx, my = wm(st["x"], st["y"])
            color = "#0074D9" if rid == int(self.robot_id.get()) else "#777"
            self.canvas.create_oval(mx - 8, my - 8, mx + 8, my + 8, fill=color, outline="")
            hx = mx + 22 * math.cos(st["yaw"])
            hy = my - 22 * math.sin(st["yaw"])
            self.canvas.create_line(mx, my, hx, hy, fill=color, width=2, arrow=tk.LAST)
            self.canvas.create_text(mx + 12, my - 14, text=f"R{rid}", fill=color)

    def _ui_loop(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is not None:
            display = self.process_frame(frame)
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            pw = max(self.panel_video.winfo_width() - 10, 100)
            ph = max(self.panel_video.winfo_height() - 28, 100)
            img.thumbnail((pw, ph))
            imgtk = ImageTk.PhotoImage(img)
            self.lbl_video.configure(image=imgtk)
            self.lbl_video.image = imgtk
        self._draw_map()
        self.root.after(33, self._ui_loop)

    def close(self):
        self.running = False
        self.abort_experiment.set()
        self.stop_robot(int(self.robot_id.get()))
        if self.cap is not None:
            self.cap.release()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = IdentificationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    main()
