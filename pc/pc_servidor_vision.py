import threading
import time
import math
import socket
import json
import heapq
from collections import deque
from pathlib import Path
from urllib.parse import urlparse
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import cv2
import numpy as np

# ============================
# CONFIG
# ============================
# Robots
ROBOT_IDS = [1, 2, 3, 10]

# ID 4 = (0,0), ID 5 = (W,0), ID 6 = (W,H), ID 7 = (0,H)
WORKSPACE_ID_TO_WORLD = {
    4: (0.0, 0.0),
    5: (1.0, 0.0),  # W se actualiza en runtime
    6: (1.0, 0.7),  # W,H se actualiza en runtime
    7: (0.0, 0.7),
}

# Discovery UDP
DISCOVERY_PORT = 37030
DISCOVERY_QUERY = b"DISCOVER_ROBOTS"
DISCOVERY_INTERVAL_S = 0.35
DISCOVERY_LISTEN_S = 0.30
ROBOT_WARN_S = 3.0
ROBOT_STALE_S = 20.0  # se marca como stale, pero no se borra la IP conocida
ROBOT_FORGET_S = 120.0
COMMAND_REDUNDANCY = 2
COMMAND_RESEND_GAP_S = 0.003

# Comandos UDP a robot
ROBOT_CMD_PORT = 44444  # todos usan este puerto (en el ESP32 tambiÃ©n)
CMD_RATE_HZ = 12

# Controladores identificados por robot.
# Valores temporales: todos usan las constantes obtenidas para robot_1.
# Cuando identifiques cada robot, cambia solo la entrada de su ID.
#
# Importante: estos PID fueron calculados desde modelos comando->velocidad.
# No se activan por defecto en el navegador posicion->PWM porque ese lazo ya
# tiene logica de orientacion, waypoints, saturaciones y evasion.
USE_IDENTIFIED_ROBOT_PID = False
IDENTIFIED_PID_GAINS = {
#    1: {               #PID del microsumo original
#        "lin_kp": 228.07679080338463,
#        "lin_ki": 37.371325439420886,
#        "lin_kd": 0.0,
#        "ang_kp": 12.993625382753669,
#        "ang_ki": 2.9970939026679195,
#        "ang_kd": 0.7356975837568529,
#    },
    1: {
        "lin_kp": 250.0,
        "lin_ki": 80.0,
        "lin_kd": 0.0,
        "ang_kp": 49.145412945779555,
        "ang_ki": 13.745376052862957,
        "ang_kd": 1.7484894130348738,
    },
    2: {
        "lin_kp": 250.0,
        "lin_ki": 80.0,
        "lin_kd": 0.0,
        "ang_kp": 49.145412945779555,
        "ang_ki": 13.745376052862957,
        "ang_kd": 1.7484894130348738,
    },
    3: {
        "lin_kp": 250.0,
        "lin_ki": 80.0,
        "lin_kd": 0.0,
        "ang_kp": 49.145412945779555,
        "ang_ki": 13.745376052862957,
        "ang_kd": 1.7484894130348738,
    },
    10: {
        "lin_kp": 250.0,
        "lin_ki": 80.0,
        "lin_kd": 0.0,
        "ang_kp": 39.33766375557281,
        "ang_ki": 12.639012969328368,
        "ang_kd": 1.156347967943816,
    },
}
PID_LINEAR_I_LIMIT = 30.0
PID_ANGULAR_I_LIMIT = 25.0

# Navegacion hacia objetivos.
# Si el objetivo queda detras o muy lateral, el robot debe girar en sitio antes
# de volver a avanzar. La histeresis evita saltos RUN/ORIENT por ruido de yaw.
ORIENT_ENTER_ANGLE_RAD = math.radians(35.0)
ORIENT_EXIT_ANGLE_RAD = math.radians(8.0)
RUN_FULL_SPEED_ANGLE_RAD = math.radians(12.0)
RUN_MIN_ALIGN_FACTOR = 0.15
ORIENT_TURN_GAIN = 2.2
ORIENT_MIN_TURN_PCT = 26.0
ORIENT_START_KICK_PCT = 34.0
ORIENT_START_KICK_S = 0.35
ORIENT_STUCK_BOOST_PCT = 34.0
ORIENT_STUCK_TIME_S = 0.45
ORIENT_STUCK_YAW_EPS_RAD = math.radians(1.0)
MOTOR_MIN_PWM_PCT = 18
RUN_MIN_LINEAR_PCT = 18

# Mapa / trayectoria
UI_CONFIG_FILE = Path(__file__).with_name("ui_config.json")
WALLS_FILE = Path(__file__).with_name("paredes.json")
NETWORK_CACHE_FILE = Path(__file__).with_name("robots_cache.json")
PATH_GRID_RES_M = 0.04
PATH_CLEARANCE_M = 0.08
PATH_WAYPOINT_REACHED_M = 0.04
WALL_FIELD_DRAW_RANGE_M = 0.18
ROBOT_POSE_MARGIN_M = 0.08
ROBOT_MAX_JUMP_M = 0.35
ROBOT_POSE_HISTORY_N = 5
ROBOT_RAW_DEADZONE_M = 0.008
ROBOT_SLOW_ALPHA = 0.18
ROBOT_FAST_ALPHA = 0.55
ROBOT_FAST_MOVE_M = 0.08
ROBOT_YAW_DEADZONE_RAD = math.radians(2.0)

# ArUco
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

ARUCO_PARAMS.minMarkerPerimeterRate = 0.003
ARUCO_PARAMS.polygonalApproxAccuracyRate = 0.06

try:
    ARUCO_PARAMS.useAruco3Detection = True
    ARUCO_PARAMS.minSideLengthCanonicalImg = 16  # Procesar a menor escala interna
    ARUCO_PARAMS.minMarkerDistanceRate = 0.005  # Permitir marcadores juntos
except AttributeError:
    pass  

ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
ARUCO_PARAMS.cornerRefinementWinSize = 5
ARUCO_PARAMS.cornerRefinementMaxIterations = 50
ARUCO_PARAMS.cornerRefinementMinAccuracy = 0.01

# Umbrales mÃ¡s robustos (si se pierden marcadores)
ARUCO_PARAMS.adaptiveThreshWinSizeMin = 5
ARUCO_PARAMS.adaptiveThreshWinSizeMax = 45
ARUCO_PARAMS.adaptiveThreshWinSizeStep = 10
ARUCO_PARAMS.adaptiveThreshConstant = 7

ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# ============================
# CALIBRACIÃ“N DE CÃMARA (Manual)
# ============================

CAM_FX = None  # Ejemplo: 650.45
CAM_FY = None  # Ejemplo: 650.45
CAM_CX = None  # Ejemplo: 320.0
CAM_CY = None  # Ejemplo: 240.0
# Coeficientes de distorsiÃ³n (k1, k2, p1, p2, k3)
CAM_DIST = None  # Ejemplo: np.array([0.1, -0.05, 0.0, 0.0, 0.0])


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MultiRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PROYECTO ROBOTICA")
        self.root.geometry("1500x860")

        self.running = True
        self.lock = threading.Lock()
        self.ui_config = self.load_ui_config()
        self._config_save_job = None

        def cfg(name, default):
            return self.ui_config.get(name, default)

        # ---------- Video ----------
        self.cap = None
        self.latest_frame = None
        self.url_camera = tk.StringVar(value=cfg("url_camera", "http://raspberry-5.local:5000/video"))

        # ---------- Workspace ----------
        self.real_width = tk.DoubleVar(value=cfg("real_width", 1.25))
        self.real_height = tk.DoubleVar(value=cfg("real_height", 1.25))
        self.homography = None

        # --- Estabilidad homografÃ­a ---
        self.homography_t = 0.0  # cuÃ¡ndo se actualizÃ³ por Ãºltima vez
        self.homography_hold_s = 1.2  # segundos que â€œaguantaâ€ el Ãºltimo H vÃ¡lido
        self.ws_center_filt = {}  # centros filtrados de IDs 4..7  (id -> np.array([x,y]))
        self.ws_last_seen = {}  # Ãºltimo tiempo visto por ID

        # --- Parallax / altura ---
        self.robot_marker_height_m = tk.DoubleVar(value=cfg("robot_marker_height_m", 0.06))  # 6 cm
        self.cam_pos_world = None  # (Cx, Cy, Cz) en metros, en coords del mundo

        # ---------- Control ----------
        self.control_active = tk.BooleanVar(value=False)
        self.selected_robot = tk.IntVar(value=cfg("selected_robot", 1))

        # Ganancias (en %)
        # Klin: Velocidad lineal
        self.k_lin_pct_per_m = tk.DoubleVar(value=cfg("k_lin_pct_per_m", 70.0))  # % por metro
        # Kang: Velocidad de giro.
        self.k_ang_pct_per_rad = tk.DoubleVar(value=cfg("k_ang_pct_per_rad", 10.0))  # % por rad
        # Vmax: Velocidad tope.
        self.vmax_pct = tk.DoubleVar(value=cfg("vmax_pct", 40.0))
        self.wspin_thresh_rad = tk.DoubleVar(value=cfg("wspin_thresh_rad", 0.55))  # ~31Â°
        self.dist_tolerance = tk.DoubleVar(value=cfg("dist_tolerance", 0.02))  # 2 cm

        # Memoria para el control Derivativo (D)
        self.prev_angle_err = {rid: 0.0 for rid in ROBOT_IDS}
        self.prev_dist_err = {rid: 0.0 for rid in ROBOT_IDS}
        self.pid_lin_i = {rid: 0.0 for rid in ROBOT_IDS}
        self.pid_ang_i = {rid: 0.0 for rid in ROBOT_IDS}
        self.orient_since = {rid: None for rid in ROBOT_IDS}
        self.orient_last_yaw = {rid: None for rid in ROBOT_IDS}
        self.orient_last_motion_t = {rid: None for rid in ROBOT_IDS}
        self.k_ang_d_pct = tk.DoubleVar(value=cfg("k_ang_d_pct", 3.5))

        # EvitaciÃ³n
        self.avoid_on = tk.BooleanVar(value=cfg("avoid_on", True))
        self.avoid_radius = tk.DoubleVar(value=cfg("avoid_radius", 0.20))  
        self.k_rep = tk.DoubleVar(value=cfg("k_rep", 0.70))  

        # ---------- Estado robots (visiÃ³n) ----------
        # robot_state[rid] = {"x":, "y":, "yaw":, "t":}
        self.robot_state = {rid: None for rid in ROBOT_IDS}
        self.robot_pose_history = {rid: deque(maxlen=ROBOT_POSE_HISTORY_N) for rid in ROBOT_IDS}

        # ---------- Objetivos ----------
        # target[rid] = (x,y) o None
        self.targets = {rid: None for rid in ROBOT_IDS}
        self.final_targets = {rid: None for rid in ROBOT_IDS}
        self.paths = {rid: [] for rid in ROBOT_IDS}
        self.choreo_running = False
        self.choreo_stop = threading.Event()
        self.choreo_thread = None
        self.choreo_button_text = tk.StringVar(value="Coreografia")

        # ---------- Paredes / rutas ----------
        self.walls = self.load_walls()
        self.wall_edit_mode = tk.BooleanVar(value=False)
        self.show_wall_field = tk.BooleanVar(value=cfg("show_wall_field", True))
        self.pending_wall_start = None
        self.path_grid_res = tk.DoubleVar(value=cfg("path_grid_res", PATH_GRID_RES_M))
        self.path_clearance = tk.DoubleVar(value=cfg("path_clearance", PATH_CLEARANCE_M))
        self.wall_field_range = tk.DoubleVar(value=cfg("wall_field_range", WALL_FIELD_DRAW_RANGE_M))

        # ---------- Red ----------
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        except OSError:
            pass

        # Discovery: escucha respuestas
        self.disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        except OSError:
            pass
        self.disc_sock.bind(("", DISCOVERY_PORT))
        self.disc_sock.settimeout(0.05)

        # Tabla de IPs descubiertas. Se conserva el ultimo endpoint valido aunque
        # pasen varios segundos sin discovery para no cortar comandos por jitter WiFi.
        # discovered[rid] = {"ip": str, "port": int, "t": float, "first_t": float, ...}
        self.discovered = self.load_network_cache()

        # === VISUALIZACIÃ“N DE FUERZAS ===
        # Guardaremos aquÃ­ los vectores calculados para dibujarlos luego
        self.vis_vectors = {rid: {'att': None, 'rep': None, 'res': None} for rid in ROBOT_IDS}

        # ---------- UI ----------
        self._setup_ui()
        self._setup_config_autosave()

        # ---------- Threads ----------
        self.th_video = threading.Thread(target=self._video_loop, daemon=True)
        self.th_video.start()

        self.th_discovery = threading.Thread(target=self._discovery_loop, daemon=True)
        self.th_discovery.start()

        self.th_control = threading.Thread(target=self._control_loop, daemon=True)
        self.th_control.start()

        # UI refresh
        self._ui_loop()

    # =========================
    # UI
    # =========================
    def _setup_ui(self):
        top = tk.Frame(self.root, bg="#ddd", pady=8)
        top.pack(side=tk.TOP, fill=tk.X)
        row1 = tk.Frame(top, bg="#ddd")
        row1.pack(side=tk.TOP, fill=tk.X, padx=2, pady=(0, 3))
        row2 = tk.Frame(top, bg="#ddd")
        row2.pack(side=tk.TOP, fill=tk.X, padx=2)
        top = row1

        tk.Label(top, text="IP cam URL:", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.url_camera, width=35).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Conectar", command=self.connect_camera, bg="#4CAF50", fg="white").pack(side=tk.LEFT,
                                                                                                    padx=4)

        tk.Label(top, text=" | W(m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.real_width, width=6).pack(side=tk.LEFT)
        tk.Label(top, text="H(m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.real_height, width=6).pack(side=tk.LEFT)

        # ========================================================
        # ### CAMPO PARA ALTURA DEL ROBOT (PARALAJE) ###
        # ========================================================
        tk.Label(top, text="Alt.Rob(m):", bg="#ddd", fg="blue").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.robot_marker_height_m, width=6).pack(side=tk.LEFT)
        # ========================================================

        tk.Checkbutton(top, text="CONTROL ON", variable=self.control_active, bg="#ddd",
                       font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Button(top, text="PARAR", command=self.stop_all, bg="red", fg="white").pack(side=tk.LEFT, padx=4)
        tk.Button(
            top,
            textvariable=self.choreo_button_text,
            command=self.toggle_choreography,
            bg="#673AB7",
            fg="white",
        ).pack(side=tk.LEFT, padx=4)

        tk.Label(top, text=" | Robot activo:", bg="#ddd").pack(side=tk.LEFT)
        ttk.Combobox(top, textvariable=self.selected_robot, values=ROBOT_IDS, width=4, state="readonly").pack(
            side=tk.LEFT)

        top = row2
        tk.Checkbutton(top, text="Evitar choques", variable=self.avoid_on, bg="#ddd").pack(side=tk.LEFT, padx=8)
        tk.Label(top, text="R(m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.avoid_radius, width=5).pack(side=tk.LEFT)

        tk.Checkbutton(top, text="Editar paredes", variable=self.wall_edit_mode, bg="#ddd").pack(side=tk.LEFT, padx=8)
        tk.Checkbutton(top, text="Campo paredes", variable=self.show_wall_field, bg="#ddd").pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Guardar paredes", command=self.save_walls).pack(side=tk.LEFT, padx=2)
        tk.Button(top, text="Deshacer pared", command=self.undo_wall).pack(side=tk.LEFT, padx=2)
        tk.Label(top, text="Clear(m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.path_clearance, width=5).pack(side=tk.LEFT)
        tk.Label(top, text="Campo(m):", bg="#ddd").pack(side=tk.LEFT, padx=(8, 0))
        tk.Entry(top, textvariable=self.wall_field_range, width=5).pack(side=tk.LEFT)

        # Ganancias
        tk.Label(top, text=" | Klin(%/m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.k_lin_pct_per_m, width=6).pack(side=tk.LEFT)
        tk.Label(top, text="Kang(%/rad):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.k_ang_pct_per_rad, width=6).pack(side=tk.LEFT)
        if USE_IDENTIFIED_ROBOT_PID:
            tk.Label(top, text="PID ident. por robot ON", bg="#ddd", fg="#006400").pack(side=tk.LEFT, padx=8)
        tk.Button(top, text="Rebuscar robots", command=self.force_robot_discovery).pack(side=tk.LEFT, padx=8)

        # Estado discovery
        self.lbl_net = tk.Label(top, text="Discovery: ...", bg="#ddd")
        self.lbl_net.pack(side=tk.RIGHT, padx=10)
        self.lbl_path = tk.Label(top, text=f"Paredes: {len(self.walls)}", bg="#ddd")
        self.lbl_path.pack(side=tk.RIGHT, padx=10)

        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Panel video
        self.panel_cam = tk.LabelFrame(main, text="Vista ")
        self.panel_cam.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.lbl_video = tk.Label(self.panel_cam, text="Sin video", bg="black", fg="white")
        self.lbl_video.pack(fill=tk.BOTH, expand=True)

        # Panel mapa
        self.panel_map = tk.LabelFrame(main, text="Mapa 2D")
        self.panel_map.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(self.panel_map, bg="white", width=540, height=640)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas.bind("<Button-1>", self.on_map_left_click)  # crear paredes en modo edicion
        self.canvas.bind("<Button-3>", self.on_map_right_click)  # objetivo robot activo
        self.canvas.bind("<Button-2>", self.on_map_middle_click)  # limpiar objetivo robot activo

    def connect_camera(self):
        url = self.url_camera.get().strip()
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(url)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

    def stop_all(self):
        self.stop_choreography(clear_targets=False)
        self.control_active.set(False)
        with self.lock:
            for rid in ROBOT_IDS:
                self.targets[rid] = None
                self.final_targets[rid] = None
                self.paths[rid] = []
                self._reset_robot_pid(rid)
        for rid in ROBOT_IDS:
            self.send_robot_cmd(rid, 0, 0)

    def toggle_choreography(self):
        if self.choreo_running:
            self.stop_choreography(clear_targets=True)
        else:
            self.start_choreography()

    def start_choreography(self):
        if self.choreo_running:
            return
        with self.lock:
            active = [rid for rid in ROBOT_IDS if self.robot_state.get(rid) is not None]
        if not active:
            self.lbl_path.config(text="Coreografia: no hay robots visibles")
            return

        self.control_active.set(True)
        self.choreo_stop.clear()
        self.choreo_running = True
        self.choreo_button_text.set("Detener coreo")
        self.choreo_thread = threading.Thread(target=self._choreography_loop, daemon=True)
        self.choreo_thread.start()

    def stop_choreography(self, clear_targets=True):
        self.choreo_stop.set()
        self.choreo_running = False
        if hasattr(self, "choreo_button_text"):
            self.choreo_button_text.set("Coreografia")
        if clear_targets:
            with self.lock:
                for rid in ROBOT_IDS:
                    self.targets[rid] = None
                    self.final_targets[rid] = None
                    self.paths[rid] = []
                    self._reset_robot_pid(rid)
            for rid in ROBOT_IDS:
                self.send_robot_cmd(rid, 0, 0)
            self.lbl_path.config(text="Coreografia detenida")

    def _choreography_formations(self, robot_ids):
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        margin = max(ROBOT_POSE_MARGIN_M + 0.04, float(self.path_clearance.get()) + 0.04, 0.12)
        cx, cy = W * 0.5, H * 0.5
        span = max(min(W, H) * 0.28, 0.12)
        span = min(span, max((W * 0.5) - margin, 0.08), max((H * 0.5) - margin, 0.08))

        square = [
            (cx - span, cy + span),
            (cx + span, cy + span),
            (cx + span, cy - span),
            (cx - span, cy - span),
        ]
        diamond = [
            (cx, cy + span),
            (cx + span, cy),
            (cx, cy - span),
            (cx - span, cy),
        ]
        swapped = list(reversed(square))
        line_y = cy
        if len(robot_ids) <= 1:
            line = [(cx, line_y)]
        else:
            line_span = min(W - 2.0 * margin, span * 2.4)
            line = [
                (cx - line_span * 0.5 + (line_span * i / max(len(robot_ids) - 1, 1)), line_y)
                for i in range(len(robot_ids))
            ]
        orbit = []
        for i in range(max(len(robot_ids), 1)):
            a = (2.0 * math.pi * i / max(len(robot_ids), 1)) + math.radians(45.0)
            orbit.append((cx + span * math.cos(a), cy + span * math.sin(a)))

        def clipped(points):
            out = []
            for x, y in points[:len(robot_ids)]:
                out.append((clamp(x, margin, W - margin), clamp(y, margin, H - margin)))
            return out

        return [
            ("cuadrado", clipped(square)),
            ("diamante", clipped(diamond)),
            ("intercambio", clipped(swapped)),
            ("fila", clipped(line)),
            ("orbita", clipped(orbit)),
        ]

    def _choreography_loop(self):
        try:
            while not self.choreo_stop.is_set():
                with self.lock:
                    active = [rid for rid in ROBOT_IDS if self.robot_state.get(rid) is not None]
                if not active:
                    self.root.after(0, lambda: self.lbl_path.config(text="Coreografia: esperando robots visibles"))
                    time.sleep(0.5)
                    continue

                formations = self._choreography_formations(active)
                for name, points in formations:
                    if self.choreo_stop.is_set():
                        break
                    for rid, (tx, ty) in zip(active, points):
                        self.set_planned_target(rid, tx, ty, update_label=False)
                    self.root.after(0, lambda n=name: self.lbl_path.config(text=f"Coreografia: {n}"))
                    if not self._wait_choreography_arrival(active, timeout_s=11.0):
                        break
                    if self.choreo_stop.wait(1.0):
                        break
        finally:
            self.choreo_running = False
            self.root.after(0, lambda: self.choreo_button_text.set("Coreografia"))

    def _wait_choreography_arrival(self, robot_ids, timeout_s=10.0):
        t0 = time.time()
        tol = max(float(self.dist_tolerance.get()) * 2.5, 0.055)
        while not self.choreo_stop.is_set() and time.time() - t0 < timeout_s:
            with self.lock:
                states = {rid: self.robot_state.get(rid) for rid in robot_ids}
                goals = {rid: self.final_targets.get(rid) for rid in robot_ids}
            pending = 0
            for rid in robot_ids:
                st = states.get(rid)
                goal = goals.get(rid)
                if st is None or goal is None:
                    pending += 1
                    continue
                if math.hypot(float(goal[0]) - st["x"], float(goal[1]) - st["y"]) > tol:
                    pending += 1
            if pending == 0:
                return True
            time.sleep(0.15)
        return not self.choreo_stop.is_set()

    def _reset_robot_pid(self, rid):
        self.prev_angle_err[rid] = 0.0
        self.prev_dist_err[rid] = 0.0
        self.pid_lin_i[rid] = 0.0
        self.pid_ang_i[rid] = 0.0
        self.orient_since[rid] = None
        self.orient_last_yaw[rid] = None
        self.orient_last_motion_t[rid] = None

    def _pid_gains_for_robot(self, rid):
        if USE_IDENTIFIED_ROBOT_PID:
            return IDENTIFIED_PID_GAINS.get(rid, IDENTIFIED_PID_GAINS[1])
        return {
            "lin_kp": float(self.k_lin_pct_per_m.get()),
            "lin_ki": 0.0,
            "lin_kd": 0.0,
            "ang_kp": float(self.k_ang_pct_per_rad.get()),
            "ang_ki": 0.0,
            "ang_kd": float(self.k_ang_d_pct.get()),
        }

    def on_map_left_click(self, event):
        if not self.wall_edit_mode.get():
            return

        wx, wy = self.map_to_world(event.x, event.y)
        if wx is None:
            return

        if self.pending_wall_start is None:
            self.pending_wall_start = (wx, wy)
            self.lbl_path.config(text="Pared: elige punto final")
            return

        x1, y1 = self.pending_wall_start
        if math.hypot(wx - x1, wy - y1) >= 0.03:
            with self.lock:
                self.walls.append({"x1": x1, "y1": y1, "x2": wx, "y2": wy})
                self._clear_all_paths_locked()
            self.save_walls()

        self.pending_wall_start = None
        self.lbl_path.config(text=f"Paredes: {len(self.walls)}")

    def on_map_right_click(self, event):
        rid = int(self.selected_robot.get())
        tx, ty = self.map_to_world(event.x, event.y)
        if tx is None:
            return
        self._reset_robot_pid(rid)
        self.set_planned_target(rid, tx, ty)

    def on_map_middle_click(self, event):
        rid = int(self.selected_robot.get())
        with self.lock:
            self.targets[rid] = None
            self.final_targets[rid] = None
            self.paths[rid] = []
            self._reset_robot_pid(rid)
        self.send_robot_cmd(rid, 0, 0)

    def load_ui_config(self):
        if not UI_CONFIG_FILE.exists():
            return {}
        try:
            data = json.loads(UI_CONFIG_FILE.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            print(f"[CONFIG] No se pudo cargar configuracion de interfaz: {exc}")
            return {}

    def _setup_config_autosave(self):
        self._persistent_vars = {
            "url_camera": self.url_camera,
            "real_width": self.real_width,
            "real_height": self.real_height,
            "robot_marker_height_m": self.robot_marker_height_m,
            "selected_robot": self.selected_robot,
            "k_lin_pct_per_m": self.k_lin_pct_per_m,
            "k_ang_pct_per_rad": self.k_ang_pct_per_rad,
            "vmax_pct": self.vmax_pct,
            "wspin_thresh_rad": self.wspin_thresh_rad,
            "dist_tolerance": self.dist_tolerance,
            "k_ang_d_pct": self.k_ang_d_pct,
            "avoid_on": self.avoid_on,
            "avoid_radius": self.avoid_radius,
            "k_rep": self.k_rep,
            "show_wall_field": self.show_wall_field,
            "path_grid_res": self.path_grid_res,
            "path_clearance": self.path_clearance,
            "wall_field_range": self.wall_field_range,
        }
        for var in self._persistent_vars.values():
            var.trace_add("write", self.schedule_ui_config_save)

    def schedule_ui_config_save(self, *_):
        if self._config_save_job is not None:
            self.root.after_cancel(self._config_save_job)
        self._config_save_job = self.root.after(500, self.save_ui_config)

    def save_ui_config(self):
        self._config_save_job = None
        data = {"version": 1}
        for name, var in self._persistent_vars.items():
            try:
                data[name] = var.get()
            except (tk.TclError, ValueError):
                return
        try:
            UI_CONFIG_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception as exc:
            print(f"[CONFIG] No se pudo guardar configuracion de interfaz: {exc}")

    def load_walls(self):
        if not WALLS_FILE.exists():
            return []
        try:
            data = json.loads(WALLS_FILE.read_text(encoding="utf-8"))
            walls = data.get("walls", data if isinstance(data, list) else [])
            clean = []
            for wall in walls:
                clean.append({
                    "x1": float(wall["x1"]),
                    "y1": float(wall["y1"]),
                    "x2": float(wall["x2"]),
                    "y2": float(wall["y2"]),
                })
            return clean
        except Exception as exc:
            print(f"[WALLS] No se pudieron cargar paredes: {exc}")
            return []

    def save_walls(self):
        with self.lock:
            walls = list(self.walls)
        data = {
            "version": 1,
            "units": "meters",
            "walls": walls,
        }
        try:
            WALLS_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.lbl_path.config(text=f"Paredes: {len(walls)} guardadas")
        except Exception as exc:
            self.lbl_path.config(text="Error guardando paredes")
            print(f"[WALLS] No se pudieron guardar paredes: {exc}")

    def load_network_cache(self):
        now = time.time()
        discovered = {rid: None for rid in ROBOT_IDS}
        if not NETWORK_CACHE_FILE.exists():
            return discovered
        try:
            data = json.loads(NETWORK_CACHE_FILE.read_text(encoding="utf-8"))
            robots = data.get("robots", {})
            for rid in ROBOT_IDS:
                item = robots.get(str(rid))
                if not item:
                    continue
                ip = str(item["ip"])
                port = int(item.get("port", ROBOT_CMD_PORT))
                discovered[rid] = {
                    "ip": ip,
                    "port": port,
                    "t": now - ROBOT_STALE_S - 1.0,
                    "first_t": now,
                    "tx_ok": 0,
                    "tx_fail": 0,
                    "last_cmd_t": 0.0,
                }
            return discovered
        except Exception as exc:
            print(f"[NET] No se pudo cargar cache de robots: {exc}")
            return discovered

    def save_network_cache(self):
        with self.lock:
            robots = {
                str(rid): {"ip": info["ip"], "port": info["port"]}
                for rid, info in self.discovered.items()
                if info is not None
            }
        data = {"version": 1, "robots": robots}
        try:
            NETWORK_CACHE_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception as exc:
            print(f"[NET] No se pudo guardar cache de robots: {exc}")

    def undo_wall(self):
        with self.lock:
            if self.walls:
                self.walls.pop()
            self.pending_wall_start = None
            self._clear_all_paths_locked()
        self.save_walls()
        self.lbl_path.config(text=f"Paredes: {len(self.walls)}")

    def _clear_all_paths_locked(self):
        for rid in ROBOT_IDS:
            self.paths[rid] = []
            self.final_targets[rid] = self.targets.get(rid)

    def set_planned_target(self, rid, tx, ty, update_label=True):
        with self.lock:
            st = self.robot_state.get(rid)

        if st is None:
            with self.lock:
                self.targets[rid] = (tx, ty)
                self.final_targets[rid] = (tx, ty)
                self.paths[rid] = []
                self._reset_robot_pid(rid)
            if update_label:
                self.lbl_path.config(text=f"R{rid}: directo, sin pose")
            return

        start = (float(st["x"]), float(st["y"]))
        goal = (float(tx), float(ty))
        path = self.plan_path(start, goal)

        with self.lock:
            self.final_targets[rid] = goal
            if path:
                self.paths[rid] = path[1:]
                self.targets[rid] = self.paths[rid][0] if self.paths[rid] else goal
            else:
                self.paths[rid] = []
                self.targets[rid] = goal
            self._reset_robot_pid(rid)

        if update_label:
            if path:
                self.lbl_path.config(text=f"R{rid}: ruta {len(path)} pts")
            else:
                self.lbl_path.config(text=f"R{rid}: sin ruta, directo")

    # =========================
    # VIDEO THREAD
    # =========================
    def _video_loop(self):
        while self.running:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    ##frame = cv2.resize(frame, (900, 675))
                    with self.lock:
                        self.latest_frame = frame
                else:
                    time.sleep(0.03)
            else:
                time.sleep(0.1)

    # =========================
    # DISCOVERY THREAD
    # =========================
    def _camera_subnet_broadcast(self):
        try:
            host = urlparse(self.url_camera.get().strip()).hostname
            if not host:
                return None
            parts = host.split(".")
            if len(parts) == 4 and all(0 <= int(p) <= 255 for p in parts):
                return ".".join(parts[:3] + ["255"])
        except Exception:
            pass
        return None

    def _discovery_endpoints(self):
        endpoints = {("255.255.255.255", DISCOVERY_PORT)}

        cam_bcast = self._camera_subnet_broadcast()
        if cam_bcast:
            endpoints.add((cam_bcast, DISCOVERY_PORT))

        with self.lock:
            known = [info for info in self.discovered.values() if info is not None]

        for info in known:
            endpoints.add((info["ip"], DISCOVERY_PORT))

        return endpoints

    def force_robot_discovery(self):
        endpoints = self._discovery_endpoints()
        for _ in range(3):
            for endpoint in endpoints:
                try:
                    self.disc_sock.sendto(DISCOVERY_QUERY, endpoint)
                except OSError:
                    pass
            time.sleep(0.03)

    def _remember_robot(self, rid, ip, port):
        now = time.time()
        changed = False
        with self.lock:
            prev = self.discovered.get(rid)
            first_t = now if prev is None else prev.get("first_t", now)
            tx_ok = 0 if prev is None else prev.get("tx_ok", 0)
            tx_fail = 0 if prev is None else prev.get("tx_fail", 0)
            changed = prev is None or prev.get("ip") != ip or prev.get("port") != port
            self.discovered[rid] = {
                "ip": ip,
                "port": port,
                "t": now,
                "first_t": first_t,
                "tx_ok": tx_ok,
                "tx_fail": tx_fail,
                "last_cmd_t": prev.get("last_cmd_t", 0.0) if prev else 0.0,
            }
        if changed:
            self.save_network_cache()

    def _discovery_loop(self):
        while self.running:
            try:
                # 1) broadcast query
                for endpoint in self._discovery_endpoints():
                    try:
                        self.disc_sock.sendto(DISCOVERY_QUERY, endpoint)
                    except OSError:
                        pass

                # 2) leer respuestas y anuncios espontaneos un ratito.
                # No salimos con el primer timeout: en WiFi los paquetes llegan con jitter.
                t_end = time.time() + DISCOVERY_LISTEN_S
                while time.time() < t_end:
                    try:
                        data, addr = self.disc_sock.recvfrom(256)
                    except socket.timeout:
                        continue

                    msg = data.decode(errors="ignore").strip()
                    # Esperado: "ROBOT_HERE ID=1 CMDPORT=44444"
                    if msg.startswith("ROBOT_HERE"):
                        rid = None
                        port = None
                        parts = msg.split()
                        for p in parts:
                            if p.startswith("ID="):
                                try:
                                    rid = int(p.split("=")[1])
                                except:
                                    rid = None
                            if p.startswith("CMDPORT="):
                                try:
                                    port = int(p.split("=")[1])
                                except:
                                    port = None

                        if rid in ROBOT_IDS and port is not None:
                            self._remember_robot(rid, addr[0], port)

            except Exception:
                pass

            time.sleep(DISCOVERY_INTERVAL_S)

    # =========================
    # UDP SEND
    # =========================
    def send_robot_cmd(self, rid, left_pct, right_pct):
        left_pct = int(clamp(left_pct, -100, 100))
        right_pct = int(clamp(right_pct, -100, 100))

        with self.lock:
            info = self.discovered.get(rid)

        if info is None:
            return  # no descubierto aÃºn

        ip = info["ip"]
        port = info["port"]
        msg = f"M {left_pct} {right_pct}".encode()
        ok = False
        fails = 0
        try:
            for n in range(COMMAND_REDUNDANCY):
                self.cmd_sock.sendto(msg, (ip, port))
                ok = True
                if n < COMMAND_REDUNDANCY - 1:
                    time.sleep(COMMAND_RESEND_GAP_S)
        except Exception:
            fails += 1

        with self.lock:
            info = self.discovered.get(rid)
            if info is not None:
                if ok:
                    info["tx_ok"] = info.get("tx_ok", 0) + 1
                    info["last_cmd_t"] = time.time()
                if fails:
                    info["tx_fail"] = info.get("tx_fail", 0) + fails

    def robot_net_status(self, rid, now=None):
        if now is None:
            now = time.time()
        with self.lock:
            info = self.discovered.get(rid)
        if info is None:
            return None, "OFF", None

        age = now - info.get("t", 0.0)
        if age <= ROBOT_WARN_S:
            return info, "OK", age
        if age <= ROBOT_STALE_S:
            return info, "WARN", age
        return info, "STALE", age

    # =========================
    # CONTROL THREAD (robots definidos en ROBOT_IDS)
    # =========================
    def _control_loop(self):
        dt = 1.0 / CMD_RATE_HZ

        # === MÃ¡quina de estados por robot (reposo real / orientar / correr / evasiÃ³n)
        # IDLE  : reposo real (sin target o reciÃ©n llegÃ³)
        # ORIENT: solo gira hasta quedar dentro de Â±10Â°
        # RUN   : navegaciÃ³n normal
        # AVOID : evasiÃ³n por repulsiÃ³n (al salir vuelve a RUN, no a ORIENT)
        if not hasattr(self, 'nav_mode'):
            self.nav_mode = {rid: "IDLE" for rid in ROBOT_IDS}
            self.prev_goal = {rid: None for rid in ROBOT_IDS}

        while self.running:
            if not self.control_active.get():
                time.sleep(0.05)
                continue

            with self.lock:
                states = {rid: self.robot_state[rid] for rid in ROBOT_IDS}
                targets = {rid: self.targets[rid] for rid in ROBOT_IDS}
                walls = list(self.walls)

            for rid in ROBOT_IDS:
                st = states[rid]
                goal = targets[rid]

                # --- Estado base por visiÃ³n ---
                if st is None:
                    self.send_robot_cmd(rid, 0, 0)
                    self._reset_robot_pid(rid)
                    continue

                # --- Reposo real (SIN objetivo) ---
                if goal is None:
                    self.send_robot_cmd(rid, 0, 0)
                    self.nav_mode[rid] = "IDLE"
                    self.prev_goal[rid] = None
                    self._reset_robot_pid(rid)
                    with self.lock:
                        self.paths[rid] = []
                        self.final_targets[rid] = None
                    continue

                # Si venimos de reposo real (IDLE) y ahora hay objetivo -> primero orientar
                if self.nav_mode.get(rid, "IDLE") == "IDLE":
                    self.nav_mode[rid] = "ORIENT"
                    self._reset_robot_pid(rid)

                # Guardar el objetivo actual (para distinguir reposo real vs cambio dinÃ¡mico)
                self.prev_goal[rid] = goal

                rx, ry, yaw = st["x"], st["y"], st["yaw"]
                gx, gy = goal

                # --- 1. LLEGADA ---
                dist_goal = math.hypot(gx - rx, gy - ry)
                with self.lock:
                    following_path = bool(self.paths[rid])
                arrival_tol = float(self.dist_tolerance.get())
                if following_path:
                    arrival_tol = max(arrival_tol, PATH_WAYPOINT_REACHED_M)
                if dist_goal < arrival_tol:
                    next_goal = None
                    with self.lock:
                        if self.paths[rid] and self.targets[rid] == self.paths[rid][0]:
                            self.paths[rid].pop(0)
                        if self.paths[rid]:
                            next_goal = self.paths[rid][0]
                            self.targets[rid] = next_goal
                        else:
                            self.targets[rid] = None
                            self.final_targets[rid] = None

                    if next_goal is not None:
                        self._reset_robot_pid(rid)
                        continue

                    self.send_robot_cmd(rid, 0, 0)

                    # Reposo real
                    self.nav_mode[rid] = "IDLE"
                    self.prev_goal[rid] = None
                    self._reset_robot_pid(rid)
                    continue

                # --- 2. ATRACCIÃ“N ---
                dist_vector = np.array([gx - rx, gy - ry], dtype=np.float32)
                norm_goal = float(np.linalg.norm(dist_vector))
                if norm_goal > 1e-6:
                    u_goal = dist_vector / norm_goal
                else:
                    u_goal = np.array([0.0, 0.0], dtype=np.float32)

                # --- 3. REPULSIÃ“N (Simple, sin tangencial) ---
                u_rep = np.array([0.0, 0.0], dtype=np.float32)

                if self.avoid_on.get():
                    d0 = float(self.avoid_radius.get())
                    krep = float(self.k_rep.get())

                    # Robots
                    for oid in ROBOT_IDS:
                        if oid == rid: continue
                        ost = states[oid]
                        if ost is None: continue

                        ox, oy = ost["x"], ost["y"]
                        dx = rx - ox
                        dy = ry - oy
                        d = math.hypot(dx, dy)

                        if 1e-6 < d < d0:
                            mag = krep * (1.0 / d - 1.0 / d0) / (d * d)
                            u_rep += mag * np.array([dx, dy], dtype=np.float32)

                    # Paredes
                    W = float(self.real_width.get())
                    H = float(self.real_height.get())
                    wall_d0 = 0.025
                    wall_k = krep * 0.6

                    if rx < wall_d0:
                        u_rep += np.array([wall_k * (1.0 / max(rx, 1e-3) - 1.0 / wall_d0), 0.0], dtype=np.float32)
                    if (W - rx) < wall_d0:
                        u_rep += np.array([-wall_k * (1.0 / max(W - rx, 1e-3) - 1.0 / wall_d0), 0.0], dtype=np.float32)
                    if ry < wall_d0:
                        u_rep += np.array([0.0, wall_k * (1.0 / max(ry, 1e-3) - 1.0 / wall_d0)], dtype=np.float32)
                    if (H - ry) < wall_d0:
                        u_rep += np.array([0.0, -wall_k * (1.0 / max(H - ry, 1e-3) - 1.0 / wall_d0)], dtype=np.float32)

                    wall_clearance = max(float(self.path_clearance.get()), 0.03)
                    wall_range = max(float(self.wall_field_range.get()), wall_clearance + 0.02)
                    for wall in walls:
                        ax, ay = wall["x1"], wall["y1"]
                        bx, by = wall["x2"], wall["y2"]
                        abx = bx - ax
                        aby = by - ay
                        den = abx * abx + aby * aby
                        if den <= 1e-12:
                            qx, qy = ax, ay
                        else:
                            t = ((rx - ax) * abx + (ry - ay) * aby) / den
                            t = clamp(t, 0.0, 1.0)
                            qx = ax + t * abx
                            qy = ay + t * aby
                        dx = rx - qx
                        dy = ry - qy
                        d = math.hypot(dx, dy)
                        if 1e-6 < d < wall_range:
                            mag = wall_k * (1.0 / d - 1.0 / wall_range) / (d * d)
                            u_rep += mag * np.array([dx, dy], dtype=np.float32)

                # Limitar repulsiÃ³n
                norm_rep = float(np.linalg.norm(u_rep))
                MAX_REPULSION = 1.0
                if norm_rep > MAX_REPULSION:
                    u_rep = (u_rep / norm_rep) * MAX_REPULSION

                # --- 4. RESULTANTE ---
                u = u_goal + u_rep

                # VISUALIZACION
                with self.lock:
                    self.vis_vectors[rid]['att'] = u_goal.copy()
                    self.vis_vectors[rid]['rep'] = u_rep.copy()
                    self.vis_vectors[rid]['res'] = u.copy()

                if float(np.linalg.norm(u)) < 1e-6:
                    self.send_robot_cmd(rid, 0, 0)
                    continue

                # --- 5. HEADINGS ---
                # Heading hacia el objetivo PURO (para ORIENT de reposo real)
                desired_heading_goal = math.atan2(float(u_goal[1]), float(u_goal[0]))

                # Heading hacia la resultante (objetivo + repulsiÃ³n) para RUN/AVOID
                desired_heading_res = math.atan2(float(u[1]), float(u[0]))

                mode = self.nav_mode.get(rid, "IDLE")

                # --- 5.1 Cambiar a modo evasiÃ³n si hay repulsiÃ³n relevante ---
                IS_SAFE_ZONE = (norm_rep < 0.15)  # tu criterio actual
                if self.avoid_on.get() and (not IS_SAFE_ZONE):
                    if mode != "AVOID":
                        self._reset_robot_pid(rid)
                    mode = "AVOID"
                else:
                    # Si estÃ¡bamos evitando y ya salimos, volvemos a RUN (NO a ORIENT)
                    if mode == "AVOID":
                        self._reset_robot_pid(rid)
                        mode = "RUN"

                # --- 5.2 Elegir heading segÃºn modo ---
                if mode == "ORIENT":
                    desired_heading = desired_heading_goal
                else:
                    desired_heading = desired_heading_res

                angle_err = wrap_pi(desired_heading - yaw)

                vmax = float(self.vmax_pct.get())

                # 1. Throttle por distancia (igual que antes)
                dist_factor = min(dist_goal / 0.15, 1.0)

                # 3. LÃ³gica por estados
                if mode == "ORIENT":
                    if abs(angle_err) <= ORIENT_EXIT_ANGLE_RAD:
                        mode = "RUN"
                        self._reset_robot_pid(rid)
                        angle_err = wrap_pi(desired_heading_res - yaw)
                        align_factor = 1.0
                    else:
                        # En ORIENT: no avanza, solo gira hasta quedar bien alineado.
                        align_factor = 0.0
                        dist_factor = 0.0  # fuerza lineal=0

                elif mode == "RUN":
                    # En RUN: aplica tu lÃ³gica normal en zona segura
                    if IS_SAFE_ZONE:
                        abs_err = abs(angle_err)
                        if abs_err <= RUN_FULL_SPEED_ANGLE_RAD:
                            align_factor = 1.0
                        elif abs_err >= ORIENT_ENTER_ANGLE_RAD:
                            align_factor = RUN_MIN_ALIGN_FACTOR
                        else:
                            span = max(ORIENT_ENTER_ANGLE_RAD - RUN_FULL_SPEED_ANGLE_RAD, 1e-6)
                            blend = (ORIENT_ENTER_ANGLE_RAD - abs_err) / span
                            align_factor = clamp(
                                RUN_MIN_ALIGN_FACTOR + (1.0 - RUN_MIN_ALIGN_FACTOR) * blend,
                                RUN_MIN_ALIGN_FACTOR,
                                1.0,
                            )
                    else:
                        # Si por algÃºn motivo estamos RUN pero aparece repulsiÃ³n,
                
                        align_factor = max(0.0, math.cos(angle_err))

                else:  # AVOID
                    # EvasiÃ³n: mantenemos coseno 
                    align_factor = max(0.0, math.cos(angle_err))

                # Guardar modo final
                self.nav_mode[rid] = mode

                prev_err = self.prev_angle_err.get(rid, 0.0)
                if USE_IDENTIFIED_ROBOT_PID:
                    gains = self._pid_gains_for_robot(rid)
                    d_err = wrap_pi(angle_err - prev_err) / max(dt, 1e-6)
                    self.pid_ang_i[rid] = clamp(
                        self.pid_ang_i[rid] + angle_err * dt,
                        -PID_ANGULAR_I_LIMIT,
                        PID_ANGULAR_I_LIMIT,
                    )
                    angular_raw = (
                        gains["ang_kp"] * angle_err
                        + gains["ang_ki"] * self.pid_ang_i[rid]
                        + gains["ang_kd"] * d_err
                    )
                else:
                    kp_ang = float(self.k_ang_pct_per_rad.get())
                    kd_ang = float(self.k_ang_d_pct.get())
                    d_err = angle_err - prev_err
                    angular_raw = (kp_ang * angle_err) + (kd_ang * d_err * 10.0)

                self.prev_angle_err[rid] = angle_err
                angular_val = clamp(angular_raw, -vmax, vmax)

                if mode == "ORIENT":
                    now_orient = time.time()
                    if self.orient_since.get(rid) is None:
                        self.orient_since[rid] = now_orient
                        self.orient_last_yaw[rid] = yaw
                        self.orient_last_motion_t[rid] = now_orient

                    last_yaw = self.orient_last_yaw.get(rid)
                    if last_yaw is None:
                        self.orient_last_yaw[rid] = yaw
                        self.orient_last_motion_t[rid] = now_orient
                    elif abs(wrap_pi(yaw - last_yaw)) >= ORIENT_STUCK_YAW_EPS_RAD:
                        self.orient_last_yaw[rid] = yaw
                        self.orient_last_motion_t[rid] = now_orient

                    orient_elapsed = now_orient - (self.orient_since.get(rid) or now_orient)
                    stuck_elapsed = now_orient - (self.orient_last_motion_t.get(rid) or now_orient)
                    min_turn = ORIENT_MIN_TURN_PCT
                    if orient_elapsed <= ORIENT_START_KICK_S:
                        min_turn = max(min_turn, ORIENT_START_KICK_PCT)
                    elif stuck_elapsed >= ORIENT_STUCK_TIME_S:
                        min_turn = max(min_turn, ORIENT_STUCK_BOOST_PCT)

                    angular_val *= ORIENT_TURN_GAIN
                    if abs(angular_val) < min_turn and abs(angle_err) > math.radians(2.0):
                        angular_val = math.copysign(min_turn, angular_val if angular_val != 0 else angle_err)
                    angular_val = clamp(angular_val, -max(vmax, min_turn), max(vmax, min_turn))
                else:
                    self.orient_since[rid] = None
                    self.orient_last_yaw[rid] = None
                    self.orient_last_motion_t[rid] = None

                # Si align_factor es bajo (robot frenado o curveando cerrado),
                if mode != "ORIENT" and align_factor < 0.5:
                    # InterpolaciÃ³n Lineal Inversa:
                    # - Si align_factor es 0.0 (Parado) -> Boost = 3.5 (Giro muy rÃ¡pido)
                    # - Si align_factor es 0.4 (Curva)  -> Boost = 1.5 (Giro alegre)
                    # - Si align_factor es 0.5 (Recto)  -> Boost = 1.0 (Normal)

                    boost = 2.5 - (align_factor * 3.0)
                    boost = max(1.0, boost)  # Nunca bajar de 1.0

                    angular_val *= boost
                    # Re-limitamos para no saturar
                    angular_val = clamp(angular_val, -vmax, vmax)

                if not USE_IDENTIFIED_ROBOT_PID:
                    raw_linear = float(self.k_lin_pct_per_m.get()) * dist_factor * align_factor
                    linear_val = clamp(raw_linear, 0, vmax)
                    self.pid_lin_i[rid] = 0.0
                    self.prev_dist_err[rid] = 0.0
                else:
                    # PID lineal: error de distancia -> comando comun u_v.
                    if mode == "ORIENT" or align_factor <= 0.0:
                        self.pid_lin_i[rid] = 0.0
                        self.prev_dist_err[rid] = 0.0
                        linear_val = 0.0
                    else:
                        gains = self._pid_gains_for_robot(rid)
                        dist_err_pid = dist_goal * align_factor
                        prev_dist_err = self.prev_dist_err.get(rid, 0.0)
                        d_dist = (dist_err_pid - prev_dist_err) / max(dt, 1e-6)
                        self.prev_dist_err[rid] = dist_err_pid
                        self.pid_lin_i[rid] = clamp(
                            self.pid_lin_i[rid] + dist_err_pid * dt,
                            -PID_LINEAR_I_LIMIT,
                            PID_LINEAR_I_LIMIT,
                        )

                        raw_linear = (
                            gains["lin_kp"] * dist_err_pid
                            + gains["lin_ki"] * self.pid_lin_i[rid]
                            + gains["lin_kd"] * d_dist
                        )
                        linear_val = clamp(raw_linear, 0, vmax)

                if (
                    mode == "RUN"
                    and align_factor >= 0.85
                    and dist_goal > max(arrival_tol * 1.5, 0.03)
                    and vmax > 0.0
                ):
                    linear_val = max(linear_val, min(vmax, RUN_MIN_LINEAR_PCT))

                # Mezclamos lineal y angular SIN usar "if error > spin_th"
                left = linear_val - angular_val
                right = linear_val + angular_val

                # Zona muerta y Clamping
                left = int(clamp(left, -100, 100))
                right = int(clamp(right, -100, 100))

                if abs(left) < MOTOR_MIN_PWM_PCT and abs(left) > 1: left = math.copysign(MOTOR_MIN_PWM_PCT, left)
                if abs(right) < MOTOR_MIN_PWM_PCT and abs(right) > 1: right = math.copysign(MOTOR_MIN_PWM_PCT, right)

                self.send_robot_cmd(rid, left, right)

            time.sleep(dt)

    # =========================
    # VISION: detecciÃ³n + homografÃ­a
    # =========================
    def _get_marker_center(self, corners_4x2):
        return np.mean(corners_4x2, axis=0)

    def _build_homography(self, ids, corners, W, H):
        """
        Usa IDs 4..7 mapeados a esquinas conocidas.
        """
        img_pts = []
        world_pts = []

        # actualizar mapeo con W,H actuales
        id2w = dict(WORKSPACE_ID_TO_WORLD)
        id2w[5] = (W, 0.0)
        id2w[6] = (W, H)
        id2w[7] = (0.0, H)

        for i, mid in enumerate(ids):
            if mid in id2w:
                c = corners[i][0]  # 4x2
                center = self._get_marker_center(c)
                img_pts.append(center)
                world_pts.append(id2w[mid])

        if len(img_pts) != 4:
            return None

        img_pts = np.array(img_pts, dtype=np.float32)
        world_pts = np.array(world_pts, dtype=np.float32)
        Hm, _ = cv2.findHomography(img_pts, world_pts, method=0)
        return Hm

    def _transform_point(self, Hm, x, y):
        pt = np.array([[[x, y]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, Hm)
        return float(out[0][0][0]), float(out[0][0][1])

    def _estimate_camera_pose_from_workspace(self, ids, corners, W, H, frame_shape):
        """
        Estima C = (Cx,Cy,Cz) en coordenadas del mundo (metros),
        usando los centros de los ArUco 4..7 (en el suelo, z=0).
        Requiere una K aproximada (mejor si calibras).
        """
        h_img, w_img = frame_shape[:2]

        # 1) Puntos 3D del mundo (z=0) para cada ID
        id2w = {
            4: (0.0, 0.0, 0.0),
            5: (W, 0.0, 0.0),
            6: (W, H, 0.0),
            7: (0.0, H, 0.0),
        }

        img_pts = []
        obj_pts = []

        for i, mid in enumerate(ids):
            if mid in id2w:
                c = corners[i][0]  # 4x2
                center = np.mean(c, axis=0)  # (2,)
                img_pts.append(center)
                obj_pts.append(id2w[mid])

        if len(img_pts) != 4:
            return None

        img_pts = np.array(img_pts, dtype=np.float32)
        obj_pts = np.array(obj_pts, dtype=np.float32)

        # 2) Matriz intrÃ­nseca (K) y DistorsiÃ³n (dist)
        if None not in (CAM_FX, CAM_FY, CAM_CX, CAM_CY, CAM_DIST):
            # Usar valores reales calibrados
            K = np.array([[CAM_FX, 0, CAM_CX],
                          [0, CAM_FY, CAM_CY],
                          [0, 0, 1]], dtype=np.float32)
            dist = np.array(CAM_DIST, dtype=np.float32)
        else:
            # Usar aproximaciÃ³n (fallback)
            f = 0.95 * w_img
            K = np.array([[f, 0, w_img / 2],
                          [0, f, h_img / 2],
                          [0, 0, 1]], dtype=np.float32)
            dist = np.zeros((5, 1), dtype=np.float32)

        # Usamos IPPE (Infinitesimal Plane-Based Pose Estimation)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None

        R, _ = cv2.Rodrigues(rvec)
        C = (-R.T @ tvec).reshape(-1)  # cÃ¡mara en coords del mundo
        if C[2] < 0.40:
            return None
        return (float(C[0]), float(C[1]), float(C[2]))

    def _parallax_correct_xy(self, x_floor, y_floor, cam_pos, h_obj):
        """
        Dado el punto que te da la homografÃ­a (intersecciÃ³n con suelo z=0),
        corrige para obtener el XY del objeto a altura h_obj (m) sobre el suelo.

        FÃ³rmula: P_h = Cxy + ((Cz - h)/Cz) * (P0 - Cxy)
        """
        if cam_pos is None:
            return x_floor, y_floor

        cx, cy, cz = cam_pos
        if cz <= (h_obj + 0.1):
            return x_floor, y_floor  # evita divisiÃ³n rara

        s = (cz - h_obj) / cz  # < 1  (trae el punto hacia la cÃ¡mara)

        # === LIMITADOR DE EXPLOSIÃ“N ===
        # Si la correcciÃ³n intenta mover el punto mÃ¡s de un 200% relativo al centro, lo ignoramos
        if abs(s) > 2.0:
            return x_floor, y_floor

        x = cx + (x_floor - cx) * s
        y = cy + (y_floor - cy) * s
        return x, y

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

    def _filtered_robot_pose(self, rid, rx_raw, ry_raw, yaw_raw, prev_st, now):
        hist = self.robot_pose_history[rid]
        if prev_st is not None and (now - prev_st.get("t", 0.0)) > 1.0:
            hist.clear()
        hist.append((rx_raw, ry_raw, yaw_raw))

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

        # === TRUCO DE AFILADO (SHARPEN KERNEL) ===
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        frame_sharp = cv2.filter2D(frame, -1, kernel)

        # Pasamos la imagen afilada al detector
        corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(frame_sharp)

        display = frame.copy()

        # refrescar states si no se ve
        now = time.time()

        # Diccionario para guardar dÃ³nde estÃ¡n las esquinas RAW (crudas) en este frame
        current_raw_corners = {}

        if ids is not None:
            ids = ids.flatten().tolist()
            cv2.aruco.drawDetectedMarkers(display, corners, np.array(ids))

            # 1. Guardar primero las posiciones RAW detectadas
            for i, mid in enumerate(ids):
                c = corners[i][0]
                center = self._get_marker_center(c)
                current_raw_corners[mid] = center  # numpy array

            # ==================================================================
            # 2. LOGICA DE HISTERESIS FUERTE PARA EL WORKSPACE (IDs 4,5,6,7)
            # ==================================================================

            # Umbral alto: El marcador debe moverse mÃ¡s de X px para ser actualizado.
            HEAVY_LOCK_THRESHOLD = 15.0
            WS_ALPHA = 0.8  # Velocidad de actualizaciÃ³n 

            for mid in [4, 5, 6, 7]:
                if mid in current_raw_corners:
                    raw_p = np.array(current_raw_corners[mid], dtype=np.float32)

                    prev = self.ws_center_filt.get(mid)

                    if prev is None:
                        # Primera vez que lo vemos: guardar directo
                        self.ws_center_filt[mid] = raw_p
                        self.ws_last_seen[mid] = now
                    else:
                        # Ya lo conocÃ­amos. Calculamos cuÃ¡nto se moviÃ³ respecto al ANCLA.
                        dist_moved = np.linalg.norm(raw_p - prev)

                        if dist_moved > HEAVY_LOCK_THRESHOLD:
                            # CAMBIO INTENCIONAL: El usuario moviÃ³ el marcador lejos.
                            # Actualizamos el filtro (suavemente para no saltar de golpe)
                            self.ws_center_filt[mid] = (WS_ALPHA * prev) + ((1.0 - WS_ALPHA) * raw_p)
                            self.ws_last_seen[mid] = now
                        else:
                            # RUIDO / VIBRACIÃ“N: El marcador se moviÃ³ poco (ej. 4px).
                            # IGNORAMOS la nueva lectura. Mantenemos 'prev' inmutable.
                            # Solo actualizamos el tiempo 'last_seen' para saber que sigue vivo.
                            self.ws_last_seen[mid] = now

            # ==================================================================
            # 3. DIBUJAR LÃNEAS AMARILLAS USANDO LOS DATOS FILTRADOS (ESTABLES)
            # ==================================================================

            pts_draw = []
            can_draw_poly = True
            for k in [4, 5, 6, 7]:
                if k in self.ws_center_filt:
                    # Convertir a entero para dibujar
                    pt = self.ws_center_filt[k].astype(int)
                    pts_draw.append(pt)
                else:
                    can_draw_poly = False
                    break

            if can_draw_poly:
                pts_np = np.array(pts_draw, np.int32).reshape((-1, 1, 2))
                cv2.polylines(display, [pts_np], True, (0, 255, 255), 3)

            # ==================================================================
            # 4. CALCULO DE HOMOGRAFÃA (Usando los centros filtrados)
            # ==================================================================

            W = float(self.real_width.get())
            H = float(self.real_height.get())

            # Verificar si todos los marcadores 4..7 han sido vistos recientemente
            # (aunque no estÃ©n en este frame exacto, usamos su memoria)
            def _recent(mid):
                return (mid in self.ws_center_filt) and (
                        (now - self.ws_last_seen.get(mid, 0)) <= self.homography_hold_s)

            Hm_new = None
            if all(_recent(k) for k in [4, 5, 6, 7]):
                img_pts = np.array([self.ws_center_filt[4],
                                    self.ws_center_filt[5],
                                    self.ws_center_filt[6],
                                    self.ws_center_filt[7]], dtype=np.float32)

                world_pts = np.array([[0.0, 0.0],
                                      [W, 0.0],
                                      [W, H],
                                      [0.0, H]], dtype=np.float32)

                Hm_new = cv2.getPerspectiveTransform(img_pts, world_pts)

                # Gating anti-saltos (Seguridad extra)
                with self.lock:
                    Hold = None if self.homography is None else self.homography.copy()

                if Hold is not None:
                    A = Hm_new / (Hm_new[2, 2] + 1e-9)
                    B = Hold / (Hold[2, 2] + 1e-9)
                    jump = float(np.linalg.norm(A - B))
                    if jump > 0.8: Hm_new = None

            # Actualizar homografÃ­a global
            with self.lock:
                if Hm_new is not None:
                    if self.homography is None:
                        self.homography = Hm_new
                    else:
                        H_ALPHA = 0.95
                        self.homography = (H_ALPHA * self.homography) + ((1.0 - H_ALPHA) * Hm_new)
                    self.homography_t = now

            # ==================================================================
            # 5. ROBOTS (IDS definidos en ROBOT_IDS)
            # ==================================================================
            with self.lock:
                Huse = self.homography

            if Huse is not None:
                cam_pos = self._estimate_camera_pose_from_workspace(ids, corners, W, H, frame.shape)
                with self.lock:
                    self.cam_pos_world = cam_pos

                for i, mid in enumerate(ids):
                    if mid in ROBOT_IDS:
                        # 1. Obtener datos crudos
                        c = corners[i][0]
                        center = self._get_marker_center(c)
                        cx, cy = float(center[0]), float(center[1])

                        p0x, p0y = self._transform_point(Huse, float(c[0][0]), float(c[0][1]))
                        p1x, p1y = self._transform_point(Huse, float(c[1][0]), float(c[1][1]))
                        rx_raw, ry_raw = self._transform_point(Huse, cx, cy)

                        h_robot = float(self.robot_marker_height_m.get())
                        with self.lock:
                            cam_pos_curr = self.cam_pos_world

                        p0x, p0y = self._parallax_correct_xy(p0x, p0y, cam_pos_curr, h_robot)
                        p1x, p1y = self._parallax_correct_xy(p1x, p1y, cam_pos_curr, h_robot)
                        rx_raw, ry_raw = self._parallax_correct_xy(rx_raw, ry_raw, cam_pos_curr, h_robot)

                        yaw_raw = math.atan2(p1y - p0y, p1x - p0x)

                        # 2. FILTRADO SUAVE (Low-Pass Filter + Zona Muerta)
                        # -------------------------------------------------
                        # Recuperar estado anterior
                        with self.lock:
                            prev_st = self.robot_state[mid]

                        if not self._valid_robot_pose(rx_raw, ry_raw, prev_st, now):
                            continue

                        final_x, final_y, final_yaw = self._filtered_robot_pose(
                            mid, rx_raw, ry_raw, yaw_raw, prev_st, now
                        )

                        # 3. Guardar estado final
                        with self.lock:
                            self.robot_state[mid] = {"x": final_x, "y": final_y, "yaw": final_yaw, "t": now}

                        # Overlay Robot (Texto con pos suavizada)
                        cv2.putText(display, f"ID:{mid}",
                                    (int(cx), int(cy) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Limpieza de viejos
        with self.lock:
            # Discovery cleanup: no borrar durante cortes cortos de WiFi; conservar
            # el ultimo endpoint para que el envio UDP pueda seguir reintentando.
            for rid in ROBOT_IDS:
                info = self.discovered[rid]
                if info is not None and (time.time() - info["t"]) > ROBOT_FORGET_S:
                    self.discovered[rid] = None

        return display

    # =========================
    # PAREDES / PLANIFICACION
    # =========================
    def _point_segment_distance(self, px, py, ax, ay, bx, by):
        abx = bx - ax
        aby = by - ay
        den = abx * abx + aby * aby
        if den <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * abx + (py - ay) * aby) / den
        t = clamp(t, 0.0, 1.0)
        qx = ax + t * abx
        qy = ay + t * aby
        return math.hypot(px - qx, py - qy)

    def _ccw(self, ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    def _segments_intersect(self, a, b, c, d):
        ax, ay = a
        bx, by = b
        cx, cy = c
        dx, dy = d
        return (self._ccw(ax, ay, cx, cy, dx, dy) != self._ccw(bx, by, cx, cy, dx, dy) and
                self._ccw(ax, ay, bx, by, cx, cy) != self._ccw(ax, ay, bx, by, dx, dy))

    def _segment_segment_distance(self, a, b, c, d):
        if self._segments_intersect(a, b, c, d):
            return 0.0
        ax, ay = a
        bx, by = b
        cx, cy = c
        dx, dy = d
        return min(
            self._point_segment_distance(ax, ay, cx, cy, dx, dy),
            self._point_segment_distance(bx, by, cx, cy, dx, dy),
            self._point_segment_distance(cx, cy, ax, ay, bx, by),
            self._point_segment_distance(dx, dy, ax, ay, bx, by),
        )

    def _point_hits_wall(self, x, y, walls, clearance):
        for wall in walls:
            d = self._point_segment_distance(x, y, wall["x1"], wall["y1"], wall["x2"], wall["y2"])
            if d <= clearance:
                return True
        return False

    def _edge_hits_wall(self, p0, p1, walls, clearance):
        for wall in walls:
            w0 = (wall["x1"], wall["y1"])
            w1 = (wall["x2"], wall["y2"])
            if self._segment_segment_distance(p0, p1, w0, w1) <= clearance:
                return True
        return False

    def _wall_repulsion_vector(self, x, y, walls, wall_k, wall_range):
        u_rep = np.array([0.0, 0.0], dtype=np.float32)
        for wall in walls:
            ax, ay = wall["x1"], wall["y1"]
            bx, by = wall["x2"], wall["y2"]
            abx = bx - ax
            aby = by - ay
            den = abx * abx + aby * aby
            if den <= 1e-12:
                qx, qy = ax, ay
            else:
                t = ((x - ax) * abx + (y - ay) * aby) / den
                t = clamp(t, 0.0, 1.0)
                qx = ax + t * abx
                qy = ay + t * aby

            dx = x - qx
            dy = y - qy
            d = math.hypot(dx, dy)
            if 1e-6 < d < wall_range:
                mag = wall_k * (1.0 / d - 1.0 / wall_range) / (d * d)
                u_rep += mag * np.array([dx, dy], dtype=np.float32)
        return u_rep

    def _smooth_path(self, path, walls, clearance):
        if len(path) <= 2:
            return path

        smooth = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if not self._edge_hits_wall(path[i], path[j], walls, clearance):
                    break
                j -= 1
            smooth.append(path[j])
            i = j
        return smooth

    def plan_path(self, start, goal):
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        res = clamp(float(self.path_grid_res.get()), 0.015, 0.20)
        clearance = clamp(float(self.path_clearance.get()), 0.0, 0.25)

        with self.lock:
            walls = list(self.walls)

        if not walls:
            return [start, goal]

        cols = max(2, int(math.ceil(W / res)) + 1)
        rows = max(2, int(math.ceil(H / res)) + 1)

        def to_cell(p):
            x, y = p
            return (
                int(clamp(round(x / res), 0, cols - 1)),
                int(clamp(round(y / res), 0, rows - 1)),
            )

        def to_world(cell):
            ci, cj = cell
            return (
                clamp(ci * res, 0.0, W),
                clamp(cj * res, 0.0, H),
            )

        start_cell = to_cell(start)
        goal_cell = to_cell(goal)

        def blocked(cell):
            if cell == start_cell or cell == goal_cell:
                return False
            x, y = to_world(cell)
            return self._point_hits_wall(x, y, walls, clearance)

        if start_cell == goal_cell:
            return [start, goal]

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start_cell))
        came_from = {}
        g_score = {start_cell: 0.0}
        visited = set()

        def heuristic(cell):
            return math.hypot(goal_cell[0] - cell[0], goal_cell[1] - cell[1]) * res

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_cell:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                cells.reverse()
                path = [start]
                path.extend(to_world(cell) for cell in cells[1:-1])
                path.append(goal)
                return self._smooth_path(path, walls, clearance)

            for di, dj in neighbors:
                nb = (current[0] + di, current[1] + dj)
                if not (0 <= nb[0] < cols and 0 <= nb[1] < rows):
                    continue
                if blocked(nb):
                    continue
                if self._edge_hits_wall(to_world(current), to_world(nb), walls, clearance):
                    continue

                step_cost = math.hypot(di, dj) * res
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(nb, float("inf")):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    heapq.heappush(open_heap, (tentative + heuristic(nb), nb))

        return None

    # =========================
    # MAPA 2D
    # =========================
    def world_to_map(self, x, y, cw, ch, W, H, margin=45):
        scale = min((cw - 2 * margin) / max(W, 1e-6), (ch - 2 * margin) / max(H, 1e-6))
        ox, oy = margin, margin

        mx = ox + x * scale

        my = (oy + H * scale) - (y * scale)

        return mx, my, scale, ox, oy

    def map_to_world(self, mx, my):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        if W <= 0 or H <= 0:
            return None, None
        margin = 45
        scale = min((cw - 2 * margin) / W, (ch - 2 * margin) / H)
        ox, oy = margin, margin

        x = (mx - ox) / scale

        y = ((oy + H * scale) - my) / scale

        x = clamp(x, 0.0, W)
        y = clamp(y, 0.0, H)
        return x, y

    def draw_wall_field(self, walls, cw, ch, W, H):
        if not walls or not self.show_wall_field.get():
            return

        wall_range = max(float(self.wall_field_range.get()), 0.01)
        dash = (4, 4)

        for wall in walls:
            x1, y1 = wall["x1"], wall["y1"]
            x2, y2 = wall["x2"], wall["y2"]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue

            nx = -dy / length
            ny = dx / length
            a1 = (x1 + nx * wall_range, y1 + ny * wall_range)
            a2 = (x2 + nx * wall_range, y2 + ny * wall_range)
            b1 = (x1 - nx * wall_range, y1 - ny * wall_range)
            b2 = (x2 - nx * wall_range, y2 - ny * wall_range)

            ma1 = self.world_to_map(a1[0], a1[1], cw, ch, W, H)
            ma2 = self.world_to_map(a2[0], a2[1], cw, ch, W, H)
            mb1 = self.world_to_map(b1[0], b1[1], cw, ch, W, H)
            mb2 = self.world_to_map(b2[0], b2[1], cw, ch, W, H)

            self.canvas.create_line(ma1[0], ma1[1], ma2[0], ma2[1], fill="#e67e22", width=1, dash=dash)
            self.canvas.create_line(mb1[0], mb1[1], mb2[0], mb2[1], fill="#e67e22", width=1, dash=dash)

            c1x, c1y, scale, _, _ = self.world_to_map(x1, y1, cw, ch, W, H)
            c2x, c2y, _, _, _ = self.world_to_map(x2, y2, cw, ch, W, H)
            r = wall_range * scale
            self.canvas.create_oval(c1x - r, c1y - r, c1x + r, c1y + r,
                                    outline="#e67e22", width=1, dash=dash)
            self.canvas.create_oval(c2x - r, c2y - r, c2x + r, c2y + r,
                                    outline="#e67e22", width=1, dash=dash)

    def draw_map(self):
        self.canvas.delete("all")

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        W = float(self.real_width.get())
        H = float(self.real_height.get())
        if W <= 0 or H <= 0 or cw < 50 or ch < 50:
            return

        margin = 45
        scale = min((cw - 2 * margin) / W, (ch - 2 * margin) / H)
        ox, oy = margin, margin

        # Campo
        self.canvas.create_rectangle(ox, oy, ox + W * scale, oy + H * scale, outline="black", width=3, fill="#f3f3f3")

        corner_labels = [
            (4, 0.0, 0.0),  # Abajo Izquierda
            (5, W, 0.0),  # Abajo Derecha
            (6, W, H),  # Arriba Derecha
            (7, 0.0, H)  # Arriba Izquierda
        ]

        for (cid, cx, cy) in corner_labels:
            # Convertimos coordenada mundo a pixel
            cmx, cmy, _, _, _ = self.world_to_map(cx, cy, cw, ch, W, H)

            # Ajustamos un poquito el texto para que no quede encima de la lÃ­nea
            # Si es la parte de abajo (cy < H/2), texto mÃ¡s abajo (+15)
            # Si es la parte de arriba, texto mÃ¡s arriba (-15)
            offset_y = 15 if cy < H / 2 else -15

            self.canvas.create_text(cmx, cmy + offset_y, text=f"ID {cid}", fill="blue", font=("Arial", 10, "bold"))

        # Targets
        with self.lock:
            targets = dict(self.targets)
            final_targets = dict(self.final_targets)
            paths = {rid: list(path) for rid, path in self.paths.items()}
            states = dict(self.robot_state)
            discovered = dict(self.discovered)
            walls = list(self.walls)

        for wall in walls:
            x1, y1 = wall["x1"], wall["y1"]
            x2, y2 = wall["x2"], wall["y2"]
            mx1, my1, _, _, _ = self.world_to_map(x1, y1, cw, ch, W, H)
            mx2, my2, _, _, _ = self.world_to_map(x2, y2, cw, ch, W, H)
            self.canvas.create_line(mx1, my1, mx2, my2, fill="#5b2c06", width=5, capstyle=tk.ROUND)

        self.draw_wall_field(walls, cw, ch, W, H)

        if self.pending_wall_start is not None:
            sx, sy = self.pending_wall_start
            smx, smy, _, _, _ = self.world_to_map(sx, sy, cw, ch, W, H)
            self.canvas.create_oval(smx - 5, smy - 5, smx + 5, smy + 5, fill="#5b2c06", outline="")

        for rid, path in paths.items():
            if not path:
                continue
            pts = []
            st = states.get(rid)
            if st is not None:
                pts.append((st["x"], st["y"]))
            pts.extend(path)
            if len(pts) >= 2:
                map_pts = []
                for px, py in pts:
                    pmx, pmy, _, _, _ = self.world_to_map(px, py, cw, ch, W, H)
                    map_pts.extend([pmx, pmy])
                self.canvas.create_line(*map_pts, fill="#0b84a5", width=2, dash=(5, 3), arrow=tk.LAST)

        for rid, goal in final_targets.items():
            if goal is None:
                continue
            gx, gy = goal
            mx, my, _, _, _ = self.world_to_map(gx, gy, cw, ch, W, H)
            self.canvas.create_oval(mx - 8, my - 8, mx + 8, my + 8, outline="red", width=2)
            self.canvas.create_text(mx, my - 16, text=f"G{rid}", fill="red")

        for rid, goal in targets.items():
            if goal is None:
                continue
            gx, gy = goal
            mx, my, _, _, _ = self.world_to_map(gx, gy, cw, ch, W, H)  # Usa la nueva funciÃ³n
            self.canvas.create_oval(mx - 4, my - 4, mx + 4, my + 4, fill="#f39c12", outline="")

        # Robots
        for rid, st in states.items():
            if st is None:
                continue
            rx, ry, yaw = st["x"], st["y"], st["yaw"]

            # Convertir a pixeles con la Y invertida
            rx_draw = clamp(rx, 0.0, W)
            ry_draw = clamp(ry, 0.0, H)
            mx, my, _, _, _ = self.world_to_map(rx_draw, ry_draw, cw, ch, W, H)

            # color simple por ID
            color = {1: "#2ecc71", 2: "#3498db", 3: "#9b59b6"}.get(rid, "green")

            self.canvas.create_oval(mx - 11, my - 11, mx + 11, my + 11, fill=color, outline="")

            ex = mx + 24 * math.cos(yaw)
            ey = my - 24 * math.sin(yaw)  

            self.canvas.create_line(mx, my, ex, ey, fill="black", width=2)
            self.canvas.create_text(mx, my + 18, text=f"R{rid}", fill="black")

            # estado red
            info = discovered.get(rid)
            if info is None:
                self.canvas.create_text(mx, my - 18, text="NO NET", fill="red")
            else:
                age = time.time() - info.get("t", 0.0)
                if age <= ROBOT_WARN_S:
                    net_color = "gray25"
                    net_text = info["ip"]
                elif age <= ROBOT_STALE_S:
                    net_color = "#b9770e"
                    net_text = f"{info['ip']} {age:.0f}s"
                else:
                    net_color = "red"
                    net_text = f"STALE {age:.0f}s"
                self.canvas.create_text(mx, my - 18, text=net_text, fill=net_color)

            # === DIBUJAR FUERZAS Y PAREDES ===
            # 1. Dibujar Zona de Paredes (RectÃ¡ngulo Rojo Tenue)
            wall_d0 = 0.025  # El mismo valor que en control
            wx0, wy0, _, _, _ = self.world_to_map(wall_d0, wall_d0, cw, ch, W, H)
            wx1, wy1, _, _, _ = self.world_to_map(W - wall_d0, H - wall_d0, cw, ch, W, H)
            self.canvas.create_rectangle(wx0, wy0, wx1, wy1, outline="red", dash=(2, 4), width=1)

            # 2. Dibujar Vectores de cada Robot
            VIS_SCALE = 40.0  # Longitud visual de las flechas (pixeles)

            with self.lock:
                vectors = dict(self.vis_vectors)

            for rid, vecs in vectors.items():
                st = self.robot_state.get(rid)
                if st is None: continue

                # PosiciÃ³n del robot en pixeles
                mx, my, _, _, _ = self.world_to_map(st["x"], st["y"], cw, ch, W, H)

                # Dibujar Radio de EvasiÃ³n (CÃ­rculo punteado)
                r_pix = float(self.avoid_radius.get()) * scale
                self.canvas.create_oval(mx - r_pix, my - r_pix, mx + r_pix, my + r_pix,
                                        outline="#FFA500", dash=(2, 2))

                # Dibujar Flechas (AtracciÃ³n, RepulsiÃ³n, Resultante)
                # Nota: En pantalla Y crece hacia abajo, en matemÃ¡ticas hacia arriba.
                # Por eso restamos vector_y (my - vy).

                if vecs['att'] is not None:
                    # AtracciÃ³n (VERDE)
                    vx, vy = vecs['att']
                    self.canvas.create_line(mx, my, mx + vx * VIS_SCALE, my - vy * VIS_SCALE,
                                            fill="green", width=2, arrow=tk.LAST)

                if vecs['rep'] is not None:
                    # RepulsiÃ³n (ROJO)
                    vx, vy = vecs['rep']
                    # Solo dibujamos si hay repulsiÃ³n significativa
                    if abs(vx) > 0.01 or abs(vy) > 0.01:
                        self.canvas.create_line(mx, my, mx + vx * VIS_SCALE, my - vy * VIS_SCALE,
                                                fill="red", width=2, arrow=tk.LAST)

                if vecs['res'] is not None:
                    # Resultante (AZUL)
                    vx, vy = vecs['res']
                    self.canvas.create_line(mx, my, mx + vx * VIS_SCALE, my - vy * VIS_SCALE,
                                            fill="blue", width=3, arrow=tk.LAST)

    # =========================
    # UI LOOP
    # =========================
    def _ui_loop(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            disc = dict(self.discovered)

        # estado discovery arriba
        now = time.time()
        parts = []
        for rid in ROBOT_IDS:
            info = disc.get(rid)
            if info is None:
                parts.append(f"R{rid}:---")
                continue
            age = now - info.get("t", 0.0)
            if age <= ROBOT_WARN_S:
                tag = "OK"
            elif age <= ROBOT_STALE_S:
                tag = f"WARN {age:.0f}s"
            else:
                tag = f"STALE {age:.0f}s"
            parts.append(f"R{rid}:{info['ip']} {tag}")
        self.lbl_net.config(text=" | ".join(parts))

        if frame is not None:
            processed = self.process_frame(frame)

            rgb = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(rgb)
            pw, ph = self.panel_cam.winfo_width(), self.panel_cam.winfo_height()
            if pw > 100 and ph > 100:
                img_pil.thumbnail((pw, ph))
            imgtk = ImageTk.PhotoImage(image=img_pil)
            self.lbl_video.configure(image=imgtk)
            self.lbl_video.image = imgtk

            self.draw_map()

        self.root.after(33, self._ui_loop)


def main():
    root = tk.Tk()
    app = MultiRobotApp(root)

    def on_close():
        app.running = False
        app.save_ui_config()
        app.stop_all()
        if app.cap:
            app.cap.release()
        root.destroy()
        import os
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
