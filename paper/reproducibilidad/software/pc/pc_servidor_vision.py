import threading
import time
import math
import socket
import json
import heapq
import random
import argparse
from collections import deque
from pathlib import Path
from urllib.parse import urlparse
import tkinter as tk
from tkinter import filedialog, ttk
from PIL import Image, ImageTk

import cv2
import numpy as np

from camera_stream import open_camera_stream, reconnect_delay
from experimentos.camera import load_camera_calibration, undistort_frame
from experimentos.localization import (
    apply_localization_alignment,
    load_localization_alignment,
)
from experimentos.scenario_runner import ScenarioRunAutomation
from experimentos.runtime import DelayedCommandDispatcher, ExperimentRecorder

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
WORKSPACE_MARKER_SIZE_M = {4: 0.056, 5: 0.056, 6: 0.071, 7: 0.071}

# Discovery UDP
DISCOVERY_PORT = 37030
DISCOVERY_QUERY = b"DISCOVER_ROBOTS"
DISCOVERY_INTERVAL_S = 0.75
DISCOVERY_LISTEN_S = 0.20
DISCOVERY_BROADCAST_INTERVAL_S = 5.0
ROBOT_WARN_S = 3.0
ROBOT_STALE_S = 20.0  # se marca como stale, pero no se borra la IP conocida
ROBOT_FORGET_S = 120.0
COMMAND_REDUNDANCY = 1
COMMAND_ACK_PROBE_INTERVAL_S = 0.0
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
REPO_ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_OUTPUT_ROOT = REPO_ROOT / "paper" / "resultados" / "raw"
DEFAULT_CAMERA_CALIBRATION = REPO_ROOT / "paper" / "configuracion_local" / "camera_calibration.json"
DEFAULT_LOCALIZATION_ALIGNMENT = (
    REPO_ROOT / "paper" / "configuracion_local" / "localization_alignment.json"
)
PAPER_SCENARIOS_FILE = REPO_ROOT / "paper" / "configuraciones" / "escenarios.json"
PATH_GRID_RES_M = 0.04
PATH_CLEARANCE_M = 0.08
PATH_WAYPOINT_REACHED_M = 0.04
GRID_POSITION_MIN_TOL_M = 0.005
GRID_POSITION_SETTLE_S = 0.8
GRID_POSITION_MAX_RETRIES = 3
GRID_POSITION_TIMEOUT_S = 90.0
WALL_FIELD_DRAW_RANGE_M = 0.18
ROBOT_POSE_MARGIN_M = 0.08
ROBOT_MAX_JUMP_M = 0.35
ROBOT_POSE_HISTORY_N = 5
ROBOT_RAW_DEADZONE_M = 0.008
ROBOT_SLOW_ALPHA = 0.18
ROBOT_FAST_ALPHA = 0.55
ROBOT_FAST_MOVE_M = 0.08
ROBOT_YAW_DEADZONE_RAD = math.radians(2.0)
ROBOT_CONTROL_POSE_TIMEOUT_S = 0.45
CAMERA_POSE_FILTER_ALPHA = 0.08
CAMERA_POSE_MAX_JUMP_M = 0.08
FSM_TANGENTIAL_GAIN = 0.85
FSM_TANGENTIAL_MAX = 0.75
FSM_AVOID_MAX_LINEAR_PCT = 24.0
FSM_CURVE_MIN_TURN_PCT = 30.0
FSM_CURVE_LINEAR_CAP_PCT = 8.0
NAV_STALL_DETECT_S = 0.60
NAV_STALL_POSITION_EPS_M = 0.008
NAV_STALL_YAW_EPS_RAD = math.radians(1.5)
NAV_STALL_RECOVERY_BURST_S = 0.35
NAV_STALL_RECOVERY_COOLDOWN_S = 0.25
NAV_STALL_TURN_PCT = 38.0
NAV_STALL_FORWARD_PCT = 35.0

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

# Respaldo para marcadores con bajo contraste, reflejos o borde parcialmente
# degradado. El detector clasico sobre CLAHE recupera casos que ArUco3 omite.
ARUCO_FALLBACK_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_FALLBACK_PARAMS.minMarkerPerimeterRate = 0.003
ARUCO_FALLBACK_PARAMS.polygonalApproxAccuracyRate = 0.08
ARUCO_FALLBACK_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
ARUCO_FALLBACK_PARAMS.cornerRefinementWinSize = 5
ARUCO_FALLBACK_PARAMS.cornerRefinementMaxIterations = 50
ARUCO_FALLBACK_PARAMS.cornerRefinementMinAccuracy = 0.01
try:
    ARUCO_FALLBACK_PARAMS.useAruco3Detection = False
except AttributeError:
    pass
ARUCO_FALLBACK_DETECTOR = cv2.aruco.ArucoDetector(
    ARUCO_DICT, ARUCO_FALLBACK_PARAMS
)
ARUCO_FALLBACK_CLAHE = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))


def merge_aruco_detections(primary_corners, primary_ids, fallback_corners, fallback_ids):
    corners = list(primary_corners or [])
    ids = [] if primary_ids is None else [int(value) for value in primary_ids.flatten()]
    found = set(ids)
    if fallback_ids is not None:
        for marker_id, marker_corners in zip(fallback_ids.flatten(), fallback_corners):
            marker_id = int(marker_id)
            if marker_id in found:
                continue
            corners.append(marker_corners)
            ids.append(marker_id)
            found.add(marker_id)
    merged_ids = (
        np.asarray(ids, dtype=np.int32).reshape(-1, 1) if ids else None
    )
    return corners, merged_ids


def fsm_tangential_component(dx, dy, distance, influence_radius, gain=FSM_TANGENTIAL_GAIN):
    if distance <= 1e-9 or distance >= influence_radius:
        return np.zeros(2, dtype=np.float32)
    proximity = clamp((influence_radius - distance) / influence_radius, 0.0, 1.0)
    magnitude = float(gain) * math.sqrt(proximity)
    return magnitude * np.asarray([-dy / distance, dx / distance], dtype=np.float32)

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
        # El hilo de video es el unico propietario del lector. La UI solo
        # publica solicitudes para evitar release/read simultaneos en OpenCV.
        self.camera_request_version = 0
        self.camera_requested_url = None
        self.camera_status = "Desconectada"
        self.camera_last_error = ""
        self.camera_wakeup = threading.Event()
        self.latest_frame = None
        self.latest_frame_seq = 0
        self.latest_frame_received_perf = None
        self.last_processed_frame_seq = -1
        self.latest_processed_display = None
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
        self.robot_marker_height_m = tk.DoubleVar(value=cfg("robot_marker_height_m", 0.09))  # 9 cm
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
        self.nav_last_pose = {rid: None for rid in ROBOT_IDS}
        self.nav_last_motion_t = {rid: None for rid in ROBOT_IDS}
        self.nav_recovery_until = {rid: 0.0 for rid in ROBOT_IDS}
        self.nav_recovery_next_t = {rid: 0.0 for rid in ROBOT_IDS}
        self.k_ang_d_pct = tk.DoubleVar(value=cfg("k_ang_d_pct", 3.5))

        # EvitaciÃ³n
        self.avoid_on = tk.BooleanVar(value=cfg("avoid_on", True))
        self.avoid_radius = tk.DoubleVar(value=cfg("avoid_radius", 0.20))  
        self.k_rep = tk.DoubleVar(value=cfg("k_rep", 0.70))  

        # ---------- Estado robots (visiÃ³n) ----------
        # robot_state[rid] = {"x":, "y":, "yaw":, "t":}
        self.robot_state = {rid: None for rid in ROBOT_IDS}
        self.latest_pose_stages = {rid: None for rid in ROBOT_IDS}
        self.robot_pose_history = {rid: deque(maxlen=ROBOT_POSE_HISTORY_N) for rid in ROBOT_IDS}
        self.robot_pose_lost = {rid: False for rid in ROBOT_IDS}

        # ---------- Objetivos ----------
        # target[rid] = (x,y) o None
        self.targets = {rid: None for rid in ROBOT_IDS}
        self.final_targets = {rid: None for rid in ROBOT_IDS}
        self.paths = {rid: [] for rid in ROBOT_IDS}
        self.choreo_running = False
        self.choreo_stop = threading.Event()
        self.choreo_thread = None
        self.choreo_button_text = tk.StringVar(value="Coreografia")

        # ---------- Instrumentacion del paper ----------
        self.experiment_scenario = tk.StringVar(value=cfg("experiment_scenario", "individual"))
        self.experiment_condition = tk.StringVar(value=cfg("experiment_condition", "base"))
        self.experiment_replicate = tk.IntVar(value=cfg("experiment_replicate", 1))
        self.experiment_controller_mode = tk.StringVar(value=cfg("experiment_controller_mode", "apf_fsm"))
        self.experiment_delay_ms = tk.DoubleVar(value=cfg("experiment_delay_ms", 0.0))
        self.experiment_jitter_ms = tk.DoubleVar(value=cfg("experiment_jitter_ms", 0.0))
        self.experiment_collision_threshold_m = tk.DoubleVar(
            value=cfg("experiment_collision_threshold_m", 0.12)
        )
        self.experiment_homography_mode = tk.StringVar(
            value=cfg("experiment_homography_mode", "estabilizada")
        )
        self.experiment_parallax_enabled = tk.BooleanVar(
            value=cfg("experiment_parallax_enabled", True)
        )
        self.experiment_pose_filter_enabled = tk.BooleanVar(
            value=cfg("experiment_pose_filter_enabled", True)
        )
        self.camera_calibration_enabled = tk.BooleanVar(
            value=cfg("camera_calibration_enabled", False)
        )
        self.camera_calibration_path = tk.StringVar(
            value=cfg("camera_calibration_path", str(DEFAULT_CAMERA_CALIBRATION))
        )
        self.localization_alignment_enabled = tk.BooleanVar(
            value=cfg("localization_alignment_enabled", True)
        )
        self.localization_alignment_path = tk.StringVar(
            value=cfg(
                "localization_alignment_path", str(DEFAULT_LOCALIZATION_ALIGNMENT)
            )
        )
        self.experiment_status = tk.StringVar(value="Registro: detenido")
        self.gt_label = tk.StringVar(value="P01")
        self.gt_x = tk.DoubleVar(value=0.0)
        self.gt_y = tk.DoubleVar(value=0.0)
        self.gt_yaw_deg = tk.DoubleVar(value=0.0)
        self.gt_duration_s = tk.DoubleVar(value=5.0)
        self.grid_step_m = tk.DoubleVar(value=cfg("grid_step_m", 0.10))
        self.grid_i = tk.IntVar(value=cfg("grid_i", 3))
        self.grid_j = tk.IntVar(value=cfg("grid_j", 3))
        self.grid_accuracy_m = tk.DoubleVar(value=cfg("grid_accuracy_m", 0.01))
        self.grid_assist = None
        self.camera_calibration = None
        self.localization_alignment = None
        self.current_camera_matrix = None
        self.control_cycle_id = 0
        self.command_seq = 0
        self.pending_command_acks = {}
        self.last_ack_probe_perf = {rid: 0.0 for rid in ROBOT_IDS}
        self.command_ack_probe_interval_s = COMMAND_ACK_PROBE_INTERVAL_S
        self.random = random.Random(2026)
        self.recorder = ExperimentRecorder(EXPERIMENT_OUTPUT_ROOT, repo_root=REPO_ROOT)
        self.command_dispatcher = DelayedCommandDispatcher(self._send_delayed_command)

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
        self.cmd_sock.bind(("", 0))
        self.cmd_sock.settimeout(0.05)
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
        self.reload_camera_calibration(silent=True)
        self.reload_localization_alignment(silent=True)

        # ---------- Threads ----------
        self.th_video = threading.Thread(target=self._video_loop, daemon=True)
        self.th_video.start()

        self.th_discovery = threading.Thread(target=self._discovery_loop, daemon=True)
        self.th_discovery.start()

        self.th_control = threading.Thread(target=self._control_loop, daemon=True)
        self.th_control.start()

        self.th_ack = threading.Thread(target=self._command_ack_loop, daemon=True)
        self.th_ack.start()

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

        tk.Checkbutton(
            top,
            text="CONTROL ON",
            variable=self.control_active,
            command=self.on_control_toggle,
            bg="#ddd",
            font=("Arial", 10, "bold"),
        ).pack(side=tk.LEFT, padx=10)
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

        exp = tk.LabelFrame(self.root, text="Experimentos del paper", bg="#eef3f8", padx=4, pady=3)
        exp.pack(side=tk.TOP, fill=tk.X, padx=3, pady=(0, 2))
        exp_row1 = tk.Frame(exp, bg="#eef3f8")
        exp_row1.pack(side=tk.TOP, fill=tk.X)
        exp_row2 = tk.Frame(exp, bg="#eef3f8")
        exp_row2.pack(side=tk.TOP, fill=tk.X, pady=(3, 0))
        exp_row3 = tk.Frame(exp, bg="#e7f4e8")
        exp_row3.pack(side=tk.TOP, fill=tk.X, pady=(3, 0))

        tk.Label(exp_row1, text="Escenario:", bg="#eef3f8").pack(side=tk.LEFT)
        ttk.Combobox(
            exp_row1,
            textvariable=self.experiment_scenario,
            values=(
                "individual", "cruce_2", "cruce_2_perpendicular",
                "latencia", "localizacion", "custom",
            ),
            width=22,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(exp_row1, text="Condicion:", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row1, textvariable=self.experiment_condition, width=13).pack(side=tk.LEFT, padx=2)
        tk.Label(exp_row1, text="Rep:", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row1, textvariable=self.experiment_replicate, width=4).pack(side=tk.LEFT)
        tk.Label(exp_row1, text="Control:", bg="#eef3f8").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Combobox(
            exp_row1,
            textvariable=self.experiment_controller_mode,
            values=("apf_fsm", "apf_puro"),
            state="readonly",
            width=10,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(exp_row1, text="Delay(ms):", bg="#eef3f8").pack(side=tk.LEFT, padx=(8, 0))
        tk.Entry(exp_row1, textvariable=self.experiment_delay_ms, width=6).pack(side=tk.LEFT)
        tk.Label(exp_row1, text="Jitter(+/-ms):", bg="#eef3f8").pack(side=tk.LEFT, padx=(6, 0))
        tk.Entry(exp_row1, textvariable=self.experiment_jitter_ms, width=6).pack(side=tk.LEFT)
        tk.Label(exp_row1, text="Colision(m):", bg="#eef3f8").pack(side=tk.LEFT, padx=(6, 0))
        tk.Entry(exp_row1, textvariable=self.experiment_collision_threshold_m, width=6).pack(side=tk.LEFT)
        tk.Button(
            exp_row1,
            text="Iniciar registro",
            command=self.start_experiment_recording,
            bg="#1565c0",
            fg="white",
        ).pack(side=tk.LEFT, padx=(10, 3))
        tk.Button(
            exp_row1,
            text="Finalizar",
            command=lambda: self.stop_experiment_recording("manual"),
        ).pack(side=tk.LEFT, padx=2)
        tk.Button(exp_row1, text="Ejecutar escenario", command=self.apply_experiment_scenario).pack(
            side=tk.LEFT, padx=3
        )
        tk.Label(exp_row1, textvariable=self.experiment_status, bg="#eef3f8", fg="#174a1f").pack(
            side=tk.RIGHT, padx=8
        )

        tk.Label(exp_row2, text="Homografia:", bg="#eef3f8").pack(side=tk.LEFT)
        ttk.Combobox(
            exp_row2,
            textvariable=self.experiment_homography_mode,
            values=("estabilizada", "cruda"),
            state="readonly",
            width=11,
        ).pack(side=tk.LEFT, padx=2)
        tk.Checkbutton(
            exp_row2, text="Calibracion intrinseca", variable=self.camera_calibration_enabled, bg="#eef3f8"
        ).pack(side=tk.LEFT, padx=4)
        tk.Checkbutton(
            exp_row2, text="Paralaje", variable=self.experiment_parallax_enabled, bg="#eef3f8"
        ).pack(side=tk.LEFT, padx=4)
        tk.Checkbutton(
            exp_row2, text="Filtro pose", variable=self.experiment_pose_filter_enabled, bg="#eef3f8"
        ).pack(side=tk.LEFT, padx=4)
        tk.Button(exp_row2, text="Cargar calibracion", command=self.choose_camera_calibration).pack(
            side=tk.LEFT, padx=(6, 10)
        )
        tk.Label(exp_row2, text="GT:", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row2, textvariable=self.gt_label, width=5).pack(side=tk.LEFT)
        tk.Label(exp_row2, text="x", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row2, textvariable=self.gt_x, width=5).pack(side=tk.LEFT)
        tk.Label(exp_row2, text="y", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row2, textvariable=self.gt_y, width=5).pack(side=tk.LEFT)
        tk.Label(exp_row2, text="yaw(deg)", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row2, textvariable=self.gt_yaw_deg, width=5).pack(side=tk.LEFT)
        tk.Label(exp_row2, text="dur(s)", bg="#eef3f8").pack(side=tk.LEFT)
        tk.Entry(exp_row2, textvariable=self.gt_duration_s, width=4).pack(side=tk.LEFT)
        tk.Button(exp_row2, text="Marcar GT", command=self.mark_ground_truth).pack(side=tk.LEFT, padx=4)

        tk.Label(exp_row3, text="Posicionador de cuadricula:", bg="#e7f4e8").pack(side=tk.LEFT)
        tk.Label(exp_row3, text="paso(m)", bg="#e7f4e8").pack(side=tk.LEFT, padx=(8, 0))
        tk.Entry(exp_row3, textvariable=self.grid_step_m, width=5).pack(side=tk.LEFT)
        tk.Label(exp_row3, text="i", bg="#e7f4e8").pack(side=tk.LEFT, padx=(8, 0))
        tk.Spinbox(exp_row3, from_=0, to=12, textvariable=self.grid_i, width=3).pack(side=tk.LEFT)
        tk.Label(exp_row3, text="j", bg="#e7f4e8").pack(side=tk.LEFT, padx=(8, 0))
        tk.Spinbox(exp_row3, from_=0, to=12, textvariable=self.grid_j, width=3).pack(side=tk.LEFT)
        tk.Label(exp_row3, text="precision(m)", bg="#e7f4e8").pack(
            side=tk.LEFT, padx=(8, 0)
        )
        tk.Entry(exp_row3, textvariable=self.grid_accuracy_m, width=5).pack(side=tk.LEFT)
        tk.Checkbutton(
            exp_row3,
            text="Alineacion XY",
            variable=self.localization_alignment_enabled,
            bg="#e7f4e8",
        ).pack(side=tk.LEFT, padx=(8, 2))
        tk.Button(
            exp_row3,
            text="Cargar alineacion",
            command=self.choose_localization_alignment,
        ).pack(side=tk.LEFT, padx=2)
        tk.Button(
            exp_row3,
            text="Mover robot al punto",
            command=self.position_selected_robot_at_grid,
            bg="#2e7d32",
            fg="white",
        ).pack(side=tk.LEFT, padx=(10, 3))
        tk.Button(
            exp_row3,
            text="Confirmar alineacion y medir",
            command=self.confirm_grid_ground_truth,
        ).pack(side=tk.LEFT, padx=3)
        tk.Label(
            exp_row3,
            text="Confirmar contra la marca fisica.",
            bg="#e7f4e8",
            fg="#315b36",
        ).pack(side=tk.LEFT, padx=10)

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
        if not url:
            with self.lock:
                self.camera_status = "URL vacia"
            return
        with self.lock:
            self.camera_request_version += 1
            self.camera_requested_url = url
            self.camera_status = "Conectando..."
            self.camera_last_error = ""
            self.latest_frame = None
            self.latest_frame_received_perf = None
            self.latest_processed_display = None
        self.camera_wakeup.set()

    def _experiment_metadata(self):
        with self.lock:
            walls = [dict(wall) for wall in self.walls]
        return {
            "scenario": self.experiment_scenario.get().strip(),
            "condition": self.experiment_condition.get().strip(),
            "replicate": int(self.experiment_replicate.get()),
            "controller_mode": self.experiment_controller_mode.get(),
            "requested_delay_ms": max(0.0, float(self.experiment_delay_ms.get())),
            "jitter_ms": max(0.0, float(self.experiment_jitter_ms.get())),
            "collision_threshold_m": max(0.01, float(self.experiment_collision_threshold_m.get())),
            "homography_mode": self.experiment_homography_mode.get(),
            "camera_calibration_enabled": bool(self.camera_calibration_enabled.get()),
            "camera_calibration_path": (
                self.camera_calibration.get("path") if self.camera_calibration else None
            ),
            "localization_alignment_enabled": bool(
                self.localization_alignment_enabled.get()
                and self.localization_alignment is not None
            ),
            "localization_alignment": (
                dict(self.localization_alignment) if self.localization_alignment else None
            ),
            "parallax_enabled": bool(self.experiment_parallax_enabled.get()),
            "pose_filter_enabled": bool(self.experiment_pose_filter_enabled.get()),
            "robot_ids": list(ROBOT_IDS),
            "workspace_width_m": float(self.real_width.get()),
            "workspace_height_m": float(self.real_height.get()),
            "workspace_marker_size_m": dict(WORKSPACE_MARKER_SIZE_M),
            "robot_marker_height_m": float(self.robot_marker_height_m.get()),
            "command_rate_hz": CMD_RATE_HZ,
            "command_redundancy": COMMAND_REDUNDANCY,
            "command_ack_probe_interval_s": self.command_ack_probe_interval_s,
            "avoid_enabled": bool(self.avoid_on.get()),
            "avoid_radius_m": float(self.avoid_radius.get()),
            "repulsion_gain": float(self.k_rep.get()),
            "fsm_tangential_gain": FSM_TANGENTIAL_GAIN,
            "fsm_tangential_max": FSM_TANGENTIAL_MAX,
            "fsm_avoid_max_linear_pct": FSM_AVOID_MAX_LINEAR_PCT,
            "fsm_curve_min_turn_pct": FSM_CURVE_MIN_TURN_PCT,
            "fsm_curve_linear_cap_pct": FSM_CURVE_LINEAR_CAP_PCT,
            "nav_stall_detect_s": NAV_STALL_DETECT_S,
            "nav_stall_recovery_burst_s": NAV_STALL_RECOVERY_BURST_S,
            "nav_stall_recovery_cooldown_s": NAV_STALL_RECOVERY_COOLDOWN_S,
            "nav_stall_turn_pct": NAV_STALL_TURN_PCT,
            "nav_stall_forward_pct": NAV_STALL_FORWARD_PCT,
            "linear_gain_pct_per_m": float(self.k_lin_pct_per_m.get()),
            "angular_gain_pct_per_rad": float(self.k_ang_pct_per_rad.get()),
            "angular_derivative_gain": float(self.k_ang_d_pct.get()),
            "vmax_pct": float(self.vmax_pct.get()),
            "distance_tolerance_m": float(self.dist_tolerance.get()),
            "grid_positioning_accuracy_m": float(self.grid_accuracy_m.get()),
            "path_clearance_m": float(self.path_clearance.get()),
            "walls": walls,
        }

    def start_experiment_recording(self):
        if self.recorder.active:
            self.experiment_status.set("Registro: ya esta activo")
            return
        try:
            session_dir = self.recorder.start(self._experiment_metadata())
        except Exception as exc:
            self.experiment_status.set(f"Error registro: {exc}")
            return
        self.experiment_status.set(f"REC: {session_dir.name}")

    def stop_experiment_recording(self, reason="completed"):
        if not self.recorder.active:
            self.experiment_status.set("Registro: detenido")
            return
        session_dir = self.recorder.stop(reason)
        self.experiment_status.set(f"Guardado: {session_dir.name if session_dir else '-'}")

    def _record_event(self, event, rid="", payload=None):
        self.recorder.event(event, robot_id=rid, payload=payload)

    def choose_camera_calibration(self):
        selected = filedialog.askopenfilename(
            title="Seleccionar calibracion intrinseca",
            filetypes=(("Calibracion JSON", "*.json"), ("Todos", "*.*")),
            initialdir=str(DEFAULT_CAMERA_CALIBRATION.parent),
        )
        if not selected:
            return
        self.camera_calibration_path.set(selected)
        self.reload_camera_calibration()

    def reload_camera_calibration(self, silent=False):
        path = Path(self.camera_calibration_path.get()).expanduser()
        try:
            self.camera_calibration = load_camera_calibration(path)
            rms = self.camera_calibration.get("rms_reprojection_error")
            if not silent:
                self.experiment_status.set(f"Calibracion OK, RMS={rms:.3f}px")
        except Exception as exc:
            self.camera_calibration = None
            if not silent:
                self.experiment_status.set(f"Sin calibracion: {exc}")

    def choose_localization_alignment(self):
        selected = filedialog.askopenfilename(
            title="Seleccionar alineacion XY",
            filetypes=(("Alineacion JSON", "*.json"), ("Todos", "*.*")),
            initialdir=str(DEFAULT_LOCALIZATION_ALIGNMENT.parent),
        )
        if not selected:
            return
        self.localization_alignment_path.set(selected)
        self.reload_localization_alignment()

    def reload_localization_alignment(self, silent=False):
        path = Path(self.localization_alignment_path.get()).expanduser()
        try:
            self.localization_alignment = load_localization_alignment(path)
            if not silent:
                dx_cm = self.localization_alignment["offset_x_m"] * 100.0
                dy_cm = self.localization_alignment["offset_y_m"] * 100.0
                self.experiment_status.set(
                    f"Alineacion XY OK: dx={dx_cm:+.2f} cm, dy={dy_cm:+.2f} cm"
                )
        except Exception as exc:
            self.localization_alignment = None
            if not silent:
                self.experiment_status.set(f"Sin alineacion XY: {exc}")

    def mark_ground_truth(self):
        if not self.recorder.active:
            self.experiment_status.set("Inicia un registro antes de marcar GT")
            return
        rid = int(self.selected_robot.get())
        payload = {
            "label": self.gt_label.get().strip(),
            "robot_id": rid,
            "x": float(self.gt_x.get()),
            "y": float(self.gt_y.get()),
            "yaw_deg": float(self.gt_yaw_deg.get()),
            "duration_s": max(0.5, float(self.gt_duration_s.get())),
        }
        self._record_event("ground_truth_start", rid, payload)
        self.experiment_status.set(
            f"GT {payload['label']}: capturando {payload['duration_s']:.1f}s"
        )

        def finish_ground_truth():
            self._record_event("ground_truth_end", rid, payload)
            if self.recorder.active:
                self.experiment_status.set(f"GT {payload['label']}: completa")

        self.root.after(int(payload["duration_s"] * 1000.0), finish_ground_truth)

    def _grid_pose_summary(self, rid):
        with self.lock:
            stage = self.latest_pose_stages.get(rid)
        if not stage:
            return "pose_stages=no_disponible"
        camera_pose = stage.get("camera_pose")
        camera_text = (
            "None"
            if camera_pose is None
            else f"({camera_pose[0]:.3f},{camera_pose[1]:.3f},{camera_pose[2]:.3f})"
        )
        return (
            f"floor=({stage['floor_x']:.3f},{stage['floor_y']:.3f}) "
            f"parallax=({stage['corrected_x']:.3f},{stage['corrected_y']:.3f}) "
            f"aligned=({stage['aligned_x']:.3f},{stage['aligned_y']:.3f}) "
            f"filtered=({stage['filtered_x']:.3f},{stage['filtered_y']:.3f}) "
            f"camera={camera_text}"
        )

    def position_selected_robot_at_grid(self):
        rid = int(self.selected_robot.get())
        try:
            step = float(self.grid_step_m.get())
            index_i = int(self.grid_i.get())
            index_j = int(self.grid_j.get())
            positioning_accuracy = float(self.grid_accuracy_m.get())
        except (tk.TclError, TypeError, ValueError):
            self.experiment_status.set("Indices o paso de cuadricula invalidos")
            return
        if step <= 0.0:
            self.experiment_status.set("El paso de cuadricula debe ser positivo")
            return
        if not GRID_POSITION_MIN_TOL_M <= positioning_accuracy <= 0.05:
            self.experiment_status.set("La precision debe estar entre 0.005 y 0.05 m")
            return

        target_x = index_i * step
        target_y = index_j * step
        width = float(self.real_width.get())
        height = float(self.real_height.get())
        safe_margin = max(0.08, float(self.path_clearance.get()))
        if not (
            safe_margin <= target_x <= width - safe_margin
            and safe_margin <= target_y <= height - safe_margin
        ):
            self.experiment_status.set(
                f"Punto ({target_x:.2f},{target_y:.2f}) fuera del margen seguro"
            )
            return

        info, network_status, _ = self.robot_net_status(rid)
        with self.lock:
            pose = self.robot_state.get(rid)
            active_targets = {
                other_id: target
                for other_id, target in self.targets.items()
                if target is not None and other_id != rid
            }
        if info is None or network_status != "OK":
            self.experiment_status.set(f"R{rid} sin conexion UDP reciente")
            return
        if pose is None or time.time() - pose.get("t", 0.0) > ROBOT_CONTROL_POSE_TIMEOUT_S:
            self.experiment_status.set(f"R{rid} sin pose reciente")
            return
        if active_targets:
            self.experiment_status.set("Deten los otros objetivos antes de posicionar GT")
            return

        self.gt_label.set(f"P_{index_i:02d}_{index_j:02d}")
        self.gt_x.set(round(target_x, 6))
        self.gt_y.set(round(target_y, 6))
        self.grid_assist = {
            "phase": "moving",
            "robot_id": rid,
            "x": target_x,
            "y": target_y,
            "started": time.monotonic(),
            "retries": 0,
            "settled_since": None,
        }
        self._record_event(
            "grid_positioning_started",
            rid,
            {"i": index_i, "j": index_j, "step_m": step, "x": target_x, "y": target_y},
        )
        print(
            f"[GRID] R{rid} inicio -> ({target_x:.3f}, {target_y:.3f}) m, "
            f"precision={positioning_accuracy:.3f} m; {self._grid_pose_summary(rid)}",
            flush=True,
        )
        self.set_planned_target(rid, target_x, target_y, update_label=False)
        self.control_active.set(True)
        self.experiment_status.set(
            f"Posicionando R{rid} en P_{index_i:02d}_{index_j:02d}..."
        )

    def _update_grid_positioning(self):
        request = self.grid_assist
        if not request or request.get("phase") not in {"moving", "settling"}:
            return
        rid = request["robot_id"]
        with self.lock:
            pose = self.robot_state.get(rid)
            target = self.targets.get(rid)
        if time.monotonic() - request["started"] > GRID_POSITION_TIMEOUT_S:
            self.command_dispatcher.clear()
            self.control_active.set(False)
            for robot_id in ROBOT_IDS:
                self.send_robot_cmd(robot_id, 0, 0)
            request["phase"] = "error"
            self._record_event("grid_positioning_timeout", rid, request)
            print(f"[GRID] R{rid} timeout; STOP", flush=True)
            self.experiment_status.set(f"R{rid}: timeout posicionando; motores detenidos")
            return
        if pose is None or time.time() - pose.get("t", 0.0) > ROBOT_CONTROL_POSE_TIMEOUT_S:
            return
        if target is not None:
            request["phase"] = "moving"
            request["settled_since"] = None
            return

        distance = math.hypot(pose["x"] - request["x"], pose["y"] - request["y"])
        positioning_tol = clamp(
            float(self.grid_accuracy_m.get()), GRID_POSITION_MIN_TOL_M, 0.05
        )
        if distance > positioning_tol:
            retries = int(request.get("retries", 0))
            if retries < GRID_POSITION_MAX_RETRIES:
                request["retries"] = retries + 1
                request["phase"] = "moving"
                request["settled_since"] = None
                self.set_planned_target(
                    rid, request["x"], request["y"], update_label=False
                )
                self.control_active.set(True)
                self._record_event(
                    "grid_positioning_retry",
                    rid,
                    {
                        "retry": request["retries"],
                        "distance_error_m": float(distance),
                    },
                )
                print(
                    f"[GRID] R{rid} reintento {request['retries']}/"
                    f"{GRID_POSITION_MAX_RETRIES}; error={distance:.4f} m; "
                    f"{self._grid_pose_summary(rid)}",
                    flush=True,
                )
                self.experiment_status.set(
                    f"R{rid}: aproximacion fina {request['retries']}/"
                    f"{GRID_POSITION_MAX_RETRIES} ({distance * 100.0:.1f} cm)"
                )
                return

            self.command_dispatcher.clear()
            self.control_active.set(False)
            for robot_id in ROBOT_IDS:
                self.send_robot_cmd(robot_id, 0, 0)
            request["phase"] = "error"
            self._record_event("grid_positioning_accuracy_failed", rid, request)
            print(
                f"[GRID] R{rid} precision fallida; error={distance:.4f} m; "
                f"{self._grid_pose_summary(rid)}; STOP",
                flush=True,
            )
            self.experiment_status.set(
                f"R{rid}: no alcanzo la precision requerida; detenido a "
                f"{distance * 100.0:.1f} cm"
            )
            return

        if request.get("phase") != "settling":
            request["phase"] = "settling"
            request["settled_since"] = time.monotonic()
            self.command_dispatcher.clear(rid)
            self.send_robot_cmd(rid, 0, 0)
            self.experiment_status.set(
                f"R{rid}: asentando dentro de {positioning_tol * 100.0:.1f} cm..."
            )
            print(
                f"[GRID] R{rid} asentando; error={distance:.4f} m; "
                f"{self._grid_pose_summary(rid)}",
                flush=True,
            )
            return
        if time.monotonic() - float(request["settled_since"]) < GRID_POSITION_SETTLE_S:
            return

        self.command_dispatcher.clear()
        self.control_active.set(False)
        for robot_id in ROBOT_IDS:
            self.send_robot_cmd(robot_id, 0, 0)
        request["phase"] = "verify"
        request["arrived_x"] = float(pose["x"])
        request["arrived_y"] = float(pose["y"])
        request["distance_error_m"] = float(distance)
        self._record_event("grid_positioning_arrived", rid, request)
        print(
            f"[GRID] R{rid} llegada estable; error={distance:.4f} m; "
            f"{self._grid_pose_summary(rid)}; STOP",
            flush=True,
        )
        self.experiment_status.set(
            f"R{rid} detenido en ({pose['x']:.3f},{pose['y']:.3f}); verifica la marca fisica"
        )

    def confirm_grid_ground_truth(self):
        request = self.grid_assist
        if not request or request.get("phase") != "verify":
            self.experiment_status.set("Primero posiciona y verifica el robot en la cuadricula")
            return
        if not self.recorder.active:
            self.experiment_status.set("Inicia el registro antes de confirmar GT")
            return
        rid = request["robot_id"]
        with self.lock:
            pose = self.robot_state.get(rid)
        if pose is None or time.time() - pose.get("t", 0.0) > ROBOT_CONTROL_POSE_TIMEOUT_S:
            self.experiment_status.set(f"R{rid} perdio la pose; no se captura GT")
            return

        distance = math.hypot(pose["x"] - request["x"], pose["y"] - request["y"])
        if distance > 0.06:
            self.experiment_status.set(
                f"R{rid} se alejo {distance * 100.0:.1f} cm; vuelve a posicionarlo"
            )
            return
        request["phase"] = "capturing"
        self._record_event("grid_ground_truth_confirmed", rid, request)
        self.mark_ground_truth()

        def finish_assisted_capture():
            if self.grid_assist is request and request.get("phase") == "capturing":
                request["phase"] = "complete"
                self.experiment_status.set(f"{self.gt_label.get()}: muestra GT completa")

        self.root.after(int(max(0.5, float(self.gt_duration_s.get())) * 1000.0), finish_assisted_capture)

    def apply_experiment_scenario(self):
        try:
            data = json.loads(PAPER_SCENARIOS_FILE.read_text(encoding="utf-8"))
            scenario_name = self.experiment_scenario.get().strip()
            scenario = data["scenarios"][scenario_name]
        except Exception as exc:
            self.experiment_status.set(f"Escenario no disponible: {exc}")
            return
        if not self.recorder.active:
            self.experiment_status.set("Inicia el registro antes de ejecutar")
            return

        tolerance = float(scenario.get("start_tolerance_m", 0.10))
        assignments = []
        with self.lock:
            states = {rid: self.robot_state.get(rid) for rid in ROBOT_IDS}
        for rid_text, specification in scenario.get("robots", {}).items():
            rid = int(rid_text)
            state = states.get(rid)
            if state is None or (time.time() - state.get("t", 0.0)) > ROBOT_CONTROL_POSE_TIMEOUT_S:
                self.experiment_status.set(f"R{rid} no tiene pose reciente")
                return
            start = specification.get("start")
            if start and math.hypot(state["x"] - start[0], state["y"] - start[1]) > tolerance:
                self.experiment_status.set(f"R{rid} fuera del inicio esperado")
                return
            goal = specification["goal"]
            assignments.append((rid, float(goal[0]), float(goal[1])))

        self._record_event(
            "scenario_started",
            payload={
                "scenario": scenario_name,
                "condition": self.experiment_condition.get().strip(),
                "replicate": int(self.experiment_replicate.get()),
                "robots": [rid for rid, _, _ in assignments],
            },
        )
        for rid, goal_x, goal_y in assignments:
            self.set_planned_target(rid, goal_x, goal_y, update_label=False)
        self.control_active.set(True)
        self.experiment_status.set(f"Ejecutando: {scenario_name}")

    def stop_all(self):
        if self.grid_assist and self.grid_assist.get("phase") == "moving":
            self.grid_assist["phase"] = "aborted"
            self._record_event("grid_positioning_aborted", self.grid_assist.get("robot_id", ""))
        self.stop_choreography(clear_targets=False)
        self.control_active.set(False)
        self.command_dispatcher.clear()
        self._record_event("global_stop")
        with self.lock:
            for rid in ROBOT_IDS:
                self.targets[rid] = None
                self.final_targets[rid] = None
                self.paths[rid] = []
                self._reset_robot_pid(rid)
        for rid in ROBOT_IDS:
            self.send_robot_cmd(rid, 0, 0)

    def on_control_toggle(self):
        if self.control_active.get():
            self._record_event("control_enabled")
            return
        self.command_dispatcher.clear()
        self._record_event("control_disabled")
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
            self.command_dispatcher.clear()
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
        self.nav_last_pose[rid] = None
        self.nav_last_motion_t[rid] = None
        self.nav_recovery_until[rid] = 0.0
        self.nav_recovery_next_t[rid] = 0.0

    def _navigation_recovery_status(self, rid, x, y, yaw, now=None):
        if now is None:
            now = time.monotonic()
        current = (float(x), float(y), float(yaw))
        previous = self.nav_last_pose.get(rid)
        last_motion = self.nav_last_motion_t.get(rid)

        progressed = previous is None
        if previous is not None:
            position_delta = math.hypot(current[0] - previous[0], current[1] - previous[1])
            yaw_delta = abs(wrap_pi(current[2] - previous[2]))
            progressed = (
                position_delta >= NAV_STALL_POSITION_EPS_M
                or yaw_delta >= NAV_STALL_YAW_EPS_RAD
            )
        if progressed:
            self.nav_last_pose[rid] = current
            self.nav_last_motion_t[rid] = now
            last_motion = now

        if now < self.nav_recovery_until.get(rid, 0.0):
            return True, False
        if now < self.nav_recovery_next_t.get(rid, 0.0):
            return False, False
        if last_motion is None:
            self.nav_last_motion_t[rid] = now
            return False, False
        if now - last_motion < NAV_STALL_DETECT_S:
            return False, False

        recovery_until = now + NAV_STALL_RECOVERY_BURST_S
        self.nav_recovery_until[rid] = recovery_until
        self.nav_recovery_next_t[rid] = recovery_until + NAV_STALL_RECOVERY_COOLDOWN_S
        return True, True

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
        self.command_dispatcher.clear(rid)
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
            "experiment_scenario": self.experiment_scenario,
            "experiment_condition": self.experiment_condition,
            "experiment_replicate": self.experiment_replicate,
            "experiment_controller_mode": self.experiment_controller_mode,
            "experiment_delay_ms": self.experiment_delay_ms,
            "experiment_jitter_ms": self.experiment_jitter_ms,
            "experiment_collision_threshold_m": self.experiment_collision_threshold_m,
            "experiment_homography_mode": self.experiment_homography_mode,
            "experiment_parallax_enabled": self.experiment_parallax_enabled,
            "experiment_pose_filter_enabled": self.experiment_pose_filter_enabled,
            "camera_calibration_enabled": self.camera_calibration_enabled,
            "camera_calibration_path": self.camera_calibration_path,
            "localization_alignment_enabled": self.localization_alignment_enabled,
            "localization_alignment_path": self.localization_alignment_path,
            "grid_step_m": self.grid_step_m,
            "grid_i": self.grid_i,
            "grid_j": self.grid_j,
            "grid_accuracy_m": self.grid_accuracy_m,
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
        self.command_dispatcher.clear(rid)
        with self.lock:
            st = self.robot_state.get(rid)

        if st is None:
            with self.lock:
                self.targets[rid] = (tx, ty)
                self.final_targets[rid] = (tx, ty)
                self.paths[rid] = []
                self._reset_robot_pid(rid)
            self._record_event(
                "target_assigned",
                rid,
                {"x": float(tx), "y": float(ty), "path_points": 0, "pose_available": False},
            )
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

        self._record_event(
            "target_assigned",
            rid,
            {"x": float(tx), "y": float(ty), "path_points": len(path), "pose_available": True},
        )

        if update_label:
            if path:
                self.lbl_path.config(text=f"R{rid}: ruta {len(path)} pts")
            else:
                self.lbl_path.config(text=f"R{rid}: sin ruta, directo")

    # =========================
    # VIDEO THREAD
    # =========================
    def _video_loop(self):
        reader = None
        active_version = -1
        retry_s = None
        retry_at = 0.0
        try:
            while self.running:
                with self.lock:
                    requested_version = self.camera_request_version
                    requested_url = self.camera_requested_url

                if requested_version != active_version:
                    if reader is not None:
                        reader.release()
                    reader = None
                    active_version = requested_version
                    retry_s = None
                    retry_at = 0.0

                if not requested_url:
                    self.camera_wakeup.wait(0.2)
                    self.camera_wakeup.clear()
                    continue

                now = time.monotonic()
                if reader is None and now < retry_at:
                    self.camera_wakeup.wait(min(0.2, retry_at - now))
                    self.camera_wakeup.clear()
                    continue

                if reader is None:
                    try:
                        reader = open_camera_stream(requested_url)
                    except Exception as exc:
                        retry_s = reconnect_delay(retry_s)
                        retry_at = time.monotonic() + retry_s
                        message = str(exc).replace("\n", " ")[:180]
                        with self.lock:
                            self.camera_status = f"Reconectando en {retry_s:.1f}s"
                            self.camera_last_error = message
                        continue
                    with self.lock:
                        self.camera_status = "Conectada"
                        self.camera_last_error = ""

                try:
                    frame = reader.read()
                    if frame is None:
                        raise ConnectionError("Frame vacio")
                except Exception as exc:
                    reader.release()
                    reader = None
                    retry_s = reconnect_delay(retry_s)
                    retry_at = time.monotonic() + retry_s
                    message = str(exc).replace("\n", " ")[:180]
                    with self.lock:
                        self.camera_status = f"Reconectando en {retry_s:.1f}s"
                        self.camera_last_error = message
                        self.latest_frame = None
                        self.latest_frame_received_perf = None
                        self.latest_processed_display = None
                    self._record_event(
                        "camera_stream_error",
                        payload={"error_type": type(exc).__name__, "message": message},
                    )
                    continue

                received_perf = time.perf_counter()
                retry_s = None
                with self.lock:
                    self.latest_frame_seq += 1
                    self.latest_frame = frame
                    self.latest_frame_received_perf = received_perf
                    self.camera_status = "Conectada"
        finally:
            if reader is not None:
                reader.release()
            with self.lock:
                self.camera_status = "Desconectada"

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

    def _discovery_endpoints(self, include_broadcast=True):
        endpoints = set()
        with self.lock:
            known = [info for info in self.discovered.values() if info is not None]

        for info in known:
            endpoints.add((info["ip"], DISCOVERY_PORT))

        if include_broadcast:
            endpoints.add(("255.255.255.255", DISCOVERY_PORT))
            cam_bcast = self._camera_subnet_broadcast()
            if cam_bcast:
                endpoints.add((cam_bcast, DISCOVERY_PORT))

        return endpoints

    def force_robot_discovery(self):
        # Una sola consulta por endpoint evita saturar el bucle UDP del ESP32.
        for endpoint in self._discovery_endpoints(include_broadcast=True):
            try:
                self.disc_sock.sendto(DISCOVERY_QUERY, endpoint)
            except OSError:
                pass

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

    def _mark_robot_network_rx(self, rid, now=None):
        if now is None:
            now = time.time()
        with self.lock:
            info = self.discovered.get(rid)
            if info is None:
                return False
            info["t"] = max(float(info.get("t", 0.0)), float(now))
            info["last_rx_t"] = float(now)
        return True

    def _discovery_loop(self):
        last_broadcast_perf = 0.0
        while self.running:
            try:
                # Los robots conocidos se consultan por unicast. El broadcast se
                # reserva para cambios de IP y nuevos robots, pues cada broadcast
                # tambien carga el receptor UDP de todos los ESP32 activos.
                now_perf = time.perf_counter()
                include_broadcast = (
                    now_perf - last_broadcast_perf >= DISCOVERY_BROADCAST_INTERVAL_S
                )
                endpoints = self._discovery_endpoints(
                    include_broadcast=include_broadcast
                )
                if include_broadcast:
                    last_broadcast_perf = now_perf
                for endpoint in endpoints:
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
    def _next_command_sequence(self):
        with self.lock:
            self.command_seq = (self.command_seq + 1) & 0x7FFFFFFF
            return self.command_seq

    def _should_request_command_ack(self, rid, sent_perf):
        interval = float(self.command_ack_probe_interval_s)
        if interval <= 0.0:
            return False
        with self.lock:
            previous = float(self.last_ack_probe_perf.get(rid, 0.0))
            if sent_perf - previous < interval:
                return False
            self.last_ack_probe_perf[rid] = float(sent_perf)
        return True

    def send_robot_cmd(self, rid, left_pct, right_pct, allow_test_delay=False):
        left_pct = int(clamp(left_pct, -100, 100))
        right_pct = int(clamp(right_pct, -100, 100))
        seq = self._next_command_sequence()
        scheduled_perf = time.perf_counter()
        requested_delay = 0.0
        jitter = 0.0
        if allow_test_delay:
            requested_delay = max(0.0, float(self.experiment_delay_ms.get()))
            jitter_limit = max(0.0, float(self.experiment_jitter_ms.get()))
            jitter = self.random.uniform(-jitter_limit, jitter_limit) if jitter_limit else 0.0
        effective_delay = max(0.0, requested_delay + jitter)
        item = {
            "robot_id": int(rid),
            "left_cmd": left_pct,
            "right_cmd": right_pct,
            "seq": seq,
            "scheduled_perf": scheduled_perf,
            "requested_delay_ms": requested_delay,
            "effective_delay_ms": effective_delay,
        }

        if allow_test_delay and effective_delay > 0.0:
            self.recorder.network(
                stage="queued",
                robot_id=rid,
                seq=seq,
                left_cmd=left_pct,
                right_cmd=right_pct,
                requested_delay_ms=requested_delay,
                actual_delay_ms="",
                scheduled_perf=scheduled_perf,
                success=1,
            )
            self.command_dispatcher.schedule(
                scheduled_perf + effective_delay / 1000.0,
                item,
            )
        else:
            self._send_robot_cmd_now(item, 0.0)
        return seq

    def _send_delayed_command(self, item, actual_delay_ms):
        self._send_robot_cmd_now(item, actual_delay_ms)

    def _send_robot_cmd_now(self, item, actual_delay_ms):
        rid = item["robot_id"]
        with self.lock:
            info = self.discovered.get(rid)

        if info is None:
            self.recorder.network(
                stage="send_error",
                robot_id=rid,
                seq=item["seq"],
                left_cmd=item["left_cmd"],
                right_cmd=item["right_cmd"],
                requested_delay_ms=item["requested_delay_ms"],
                actual_delay_ms=actual_delay_ms,
                scheduled_perf=item["scheduled_perf"],
                success=0,
                error="robot_not_discovered",
            )
            return

        ip = info["ip"]
        port = info["port"]
        ok = False
        fails = 0
        error = ""
        sent_perf = time.perf_counter()
        ack_requested = self._should_request_command_ack(rid, sent_perf)
        if ack_requested:
            msg = f"M {item['left_cmd']} {item['right_cmd']} S {item['seq']}".encode()
            with self.lock:
                self.pending_command_acks[item["seq"]] = {
                    "robot_id": rid,
                    "sent_perf": sent_perf,
                    "left_cmd": item["left_cmd"],
                    "right_cmd": item["right_cmd"],
                }
        else:
            msg = f"M {item['left_cmd']} {item['right_cmd']}".encode()
        try:
            for n in range(COMMAND_REDUNDANCY):
                self.cmd_sock.sendto(msg, (ip, port))
                ok = True
                if n < COMMAND_REDUNDANCY - 1:
                    time.sleep(COMMAND_RESEND_GAP_S)
        except Exception as exc:
            fails += 1
            error = type(exc).__name__

        with self.lock:
            info = self.discovered.get(rid)
            if not ok and ack_requested:
                self.pending_command_acks.pop(item["seq"], None)
            if info is not None:
                if ok:
                    info["tx_ok"] = info.get("tx_ok", 0) + 1
                    info["last_cmd_t"] = time.time()
                if fails:
                    info["tx_fail"] = info.get("tx_fail", 0) + fails

        self.recorder.network(
            stage="sent" if ok else "send_error",
            robot_id=rid,
            seq=item["seq"],
            left_cmd=item["left_cmd"],
            right_cmd=item["right_cmd"],
            requested_delay_ms=item["requested_delay_ms"],
            actual_delay_ms=actual_delay_ms,
            scheduled_perf=item["scheduled_perf"],
            sent_perf=sent_perf,
            ack_requested=1 if ack_requested else 0,
            success=1 if ok else 0,
            error=error,
        )

    def _command_ack_loop(self):
        while self.running:
            try:
                data, _ = self.cmd_sock.recvfrom(256)
            except socket.timeout:
                data = None
            except OSError:
                break
            if data:
                message = data.decode(errors="ignore").strip()
                if message.startswith("ACK "):
                    values = {}
                    for token in message.split()[1:]:
                        if "=" in token:
                            key, value = token.split("=", 1)
                            values[key] = value
                    try:
                        rid = int(values["ID"])
                        seq = int(values["SEQ"])
                        esp_rx_ms = int(values.get("RXMS", "0"))
                    except (KeyError, ValueError):
                        continue
                    self._mark_robot_network_rx(rid)
                    ack_perf = time.perf_counter()
                    with self.lock:
                        pending = self.pending_command_acks.pop(seq, None)
                    duplicate = pending is None
                    rtt_ms = "" if duplicate else (ack_perf - pending["sent_perf"]) * 1000.0
                    self.recorder.network(
                        stage="ack",
                        robot_id=rid,
                        seq=seq,
                        left_cmd="" if duplicate else pending["left_cmd"],
                        right_cmd="" if duplicate else pending["right_cmd"],
                        ack_perf=ack_perf,
                        rtt_ms=rtt_ms,
                        esp_rx_ms=esp_rx_ms,
                        duplicate=1 if duplicate else 0,
                        success=1,
                    )

            cutoff = time.perf_counter() - 5.0
            with self.lock:
                expired = [
                    seq for seq, pending in self.pending_command_acks.items()
                    if pending["sent_perf"] < cutoff
                ]
                for seq in expired:
                    self.pending_command_acks.pop(seq, None)

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
    def _record_control_sample(
        self,
        cycle_id,
        rid,
        previous_state,
        state,
        pose,
        waypoint,
        final_goal,
        distance_error,
        desired_heading,
        angle_error,
        u_goal,
        u_rep,
        u_result,
        repulsion_norm,
        align_factor,
        linear_cmd,
        angular_cmd,
        left_cmd,
        right_cmd,
    ):
        if not self.recorder.active:
            return
        now = time.time()
        self.recorder.control(
            cycle_id=cycle_id,
            robot_id=rid,
            state=state,
            previous_state=previous_state,
            x=pose["x"],
            y=pose["y"],
            yaw=pose["yaw"],
            pose_age_ms=max(0.0, (now - pose.get("t", now)) * 1000.0),
            waypoint_x=waypoint[0] if waypoint else "",
            waypoint_y=waypoint[1] if waypoint else "",
            final_goal_x=final_goal[0] if final_goal else "",
            final_goal_y=final_goal[1] if final_goal else "",
            distance_error=distance_error,
            desired_heading=desired_heading,
            angle_error=angle_error,
            u_att_x=float(u_goal[0]),
            u_att_y=float(u_goal[1]),
            u_rep_x=float(u_rep[0]),
            u_rep_y=float(u_rep[1]),
            u_res_x=float(u_result[0]),
            u_res_y=float(u_result[1]),
            repulsion_norm=repulsion_norm,
            align_factor=align_factor,
            linear_cmd=linear_cmd,
            angular_cmd=angular_cmd,
            left_cmd=left_cmd,
            right_cmd=right_cmd,
            controller_mode=self.experiment_controller_mode.get(),
            avoid_enabled=1 if self.avoid_on.get() else 0,
            requested_delay_ms=max(0.0, float(self.experiment_delay_ms.get())),
            jitter_ms=max(0.0, float(self.experiment_jitter_ms.get())),
        )

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
                final_targets = {rid: self.final_targets[rid] for rid in ROBOT_IDS}
                walls = list(self.walls)

            self.control_cycle_id += 1
            cycle_id = self.control_cycle_id

            for rid in ROBOT_IDS:
                st = states[rid]
                goal = targets[rid]
                final_goal = final_targets[rid]

                # --- Estado base por visiÃ³n ---
                pose_is_stale = st is None or (time.time() - st.get("t", 0.0)) > ROBOT_CONTROL_POSE_TIMEOUT_S
                if pose_is_stale:
                    self.command_dispatcher.clear(rid)
                    self.send_robot_cmd(rid, 0, 0)
                    self._reset_robot_pid(rid)
                    if not self.robot_pose_lost[rid]:
                        self.robot_pose_lost[rid] = True
                        self._record_event("safety_pose_lost", rid)
                    continue
                if self.robot_pose_lost[rid]:
                    self.robot_pose_lost[rid] = False
                    self._record_event("pose_recovered", rid)

                # --- Reposo real (SIN objetivo) ---
                if goal is None:
                    self.command_dispatcher.clear(rid)
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
                    # La lista incluye el objetivo actual. Solo hay un waypoint
                    # intermedio cuando quedan al menos dos puntos por recorrer.
                    following_path = len(self.paths[rid]) > 1
                arrival_tol = float(self.dist_tolerance.get())
                grid_request = self.grid_assist
                if (
                    grid_request
                    and grid_request.get("phase") == "moving"
                    and grid_request.get("robot_id") == rid
                ):
                    requested_accuracy = clamp(
                        float(self.grid_accuracy_m.get()),
                        GRID_POSITION_MIN_TOL_M,
                        0.05,
                    )
                    arrival_tol = min(
                        arrival_tol,
                        requested_accuracy,
                    )
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
                        self.command_dispatcher.clear(rid)
                        self._reset_robot_pid(rid)
                        continue

                    self.command_dispatcher.clear(rid)
                    self.send_robot_cmd(rid, 0, 0)

                    # Reposo real
                    self.nav_mode[rid] = "IDLE"
                    self.prev_goal[rid] = None
                    self._reset_robot_pid(rid)
                    self._record_event(
                        "target_reached",
                        rid,
                        {
                            "x": float(rx),
                            "y": float(ry),
                            "goal_x": float(gx),
                            "goal_y": float(gy),
                            "distance_error_m": float(dist_goal),
                        },
                    )
                    continue

                # --- 2. ATRACCIÃ“N ---
                dist_vector = np.array([gx - rx, gy - ry], dtype=np.float32)
                norm_goal = float(np.linalg.norm(dist_vector))
                if norm_goal > 1e-6:
                    u_goal = dist_vector / norm_goal
                else:
                    u_goal = np.array([0.0, 0.0], dtype=np.float32)

                # --- 3. REPULSIÃ“N RADIAL + ACCION TANGENCIAL DE LA FSM ---
                u_rep = np.array([0.0, 0.0], dtype=np.float32)
                u_tangent = np.array([0.0, 0.0], dtype=np.float32)
                controller_mode = self.experiment_controller_mode.get()

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
                            if controller_mode == "apf_fsm":
                                u_tangent += fsm_tangential_component(dx, dy, d, d0)

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

                tangent_norm = float(np.linalg.norm(u_tangent))
                if tangent_norm > FSM_TANGENTIAL_MAX:
                    u_tangent = (u_tangent / tangent_norm) * FSM_TANGENTIAL_MAX
                u_rep += u_tangent
                norm_rep = float(np.linalg.norm(u_rep))

                # --- 4. RESULTANTE ---
                u = u_goal + u_rep

                # VISUALIZACION
                with self.lock:
                    self.vis_vectors[rid]['att'] = u_goal.copy()
                    self.vis_vectors[rid]['rep'] = u_rep.copy()
                    self.vis_vectors[rid]['res'] = u.copy()

                if float(np.linalg.norm(u)) < 1e-6:
                    self.command_dispatcher.clear(rid)
                    self.send_robot_cmd(rid, 0, 0)
                    continue

                # --- 5. HEADINGS ---
                # Heading hacia el objetivo PURO (para ORIENT de reposo real)
                desired_heading_goal = math.atan2(float(u_goal[1]), float(u_goal[0]))

                # Heading hacia la resultante (objetivo + repulsiÃ³n) para RUN/AVOID
                desired_heading_res = math.atan2(float(u[1]), float(u[0]))

                previous_mode = self.nav_mode.get(rid, "IDLE")
                mode = previous_mode
                IS_SAFE_ZONE = norm_rep < 0.15
                vmax = float(self.vmax_pct.get())
                dist_factor = min(dist_goal / 0.15, 1.0)

                if controller_mode == "apf_puro":
                    # Baseline de ablacion: sigue continuamente el vector APF sin
                    # estados ORIENT/AVOID ni sus acciones minimas especiales.
                    mode = "APF"
                    desired_heading = desired_heading_res
                    angle_err = wrap_pi(desired_heading - yaw)
                    align_factor = max(0.0, math.cos(angle_err))
                else:
                    # Control propuesto: APF acoplado a la maquina de estados.
                    if self.avoid_on.get() and not IS_SAFE_ZONE:
                        if mode != "AVOID":
                            self._reset_robot_pid(rid)
                        mode = "AVOID"
                    elif mode == "AVOID":
                        self._reset_robot_pid(rid)
                        mode = "RUN"

                    desired_heading = desired_heading_goal if mode == "ORIENT" else desired_heading_res
                    angle_err = wrap_pi(desired_heading - yaw)

                    if mode == "ORIENT":
                        if abs(angle_err) <= ORIENT_EXIT_ANGLE_RAD:
                            mode = "RUN"
                            self._reset_robot_pid(rid)
                            desired_heading = desired_heading_res
                            angle_err = wrap_pi(desired_heading - yaw)
                            align_factor = 1.0
                        else:
                            align_factor = 0.0
                            dist_factor = 0.0
                    elif mode == "RUN":
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
                            align_factor = max(0.0, math.cos(angle_err))
                    else:
                        align_factor = max(0.0, math.cos(angle_err))

                self.nav_mode[rid] = mode
                if mode != previous_mode:
                    self._record_event(
                        "state_transition",
                        rid,
                        {"from": previous_mode, "to": mode, "controller_mode": controller_mode},
                    )

                nav_recovery_active = False
                nav_recovery_started = False
                if controller_mode == "apf_fsm" and mode in {"RUN", "AVOID"}:
                    nav_recovery_active, nav_recovery_started = (
                        self._navigation_recovery_status(rid, rx, ry, yaw)
                    )

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

                if mode == "AVOID":
                    linear_val = min(linear_val, FSM_AVOID_MAX_LINEAR_PCT)

                if (
                    controller_mode == "apf_fsm"
                    and mode in {"RUN", "AVOID"}
                    and abs(angle_err) >= ORIENT_ENTER_ANGLE_RAD
                ):
                    angular_val = math.copysign(
                        max(abs(angular_val), FSM_CURVE_MIN_TURN_PCT),
                        angular_val if angular_val != 0.0 else angle_err,
                    )
                    angular_val = clamp(angular_val, -vmax, vmax)
                    linear_val = min(linear_val, FSM_CURVE_LINEAR_CAP_PCT)

                if nav_recovery_active:
                    if abs(angle_err) > math.radians(5.0):
                        linear_val = 0.0
                        angular_val = math.copysign(
                            max(abs(angular_val), NAV_STALL_TURN_PCT),
                            angular_val if angular_val != 0.0 else angle_err,
                        )
                        angular_val = clamp(
                            angular_val,
                            -max(vmax, NAV_STALL_TURN_PCT),
                            max(vmax, NAV_STALL_TURN_PCT),
                        )
                        recovery_action = "turn"
                    else:
                        linear_val = max(
                            linear_val,
                            min(vmax, NAV_STALL_FORWARD_PCT),
                        )
                        recovery_action = "forward"
                    if nav_recovery_started:
                        self._record_event(
                            "navigation_stall_recovery",
                            rid,
                            {
                                "action": recovery_action,
                                "distance_error_m": float(dist_goal),
                                "angle_error_rad": float(angle_err),
                                "mode": mode,
                            },
                        )

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

                self._record_control_sample(
                    cycle_id,
                    rid,
                    previous_mode,
                    mode,
                    st,
                    goal,
                    final_goal,
                    dist_goal,
                    desired_heading,
                    angle_err,
                    u_goal,
                    u_rep,
                    u,
                    norm_rep,
                    align_factor,
                    linear_val,
                    angular_val,
                    left,
                    right,
                )
                self.send_robot_cmd(rid, left, right, allow_test_delay=True)

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

    def _estimate_camera_pose_from_workspace(
        self,
        ids,
        corners,
        W,
        H,
        frame_shape,
        camera_matrix=None,
        distortion=None,
        image_centers=None,
    ):
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

        if image_centers is not None and all(mid in image_centers for mid in id2w):
            for mid, world_point in id2w.items():
                img_pts.append(np.asarray(image_centers[mid], dtype=np.float32))
                obj_pts.append(world_point)
        else:
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
        if camera_matrix is not None:
            K = np.asarray(camera_matrix, dtype=np.float32)
            dist = np.zeros((5, 1), dtype=np.float32) if distortion is None else np.asarray(
                distortion, dtype=np.float32
            )
        elif None not in (CAM_FX, CAM_FY, CAM_CX, CAM_CY, CAM_DIST):
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


    def process_frame(self, frame, frame_seq=0, frame_received_perf=None):
        process_start = time.perf_counter()
        now = time.time()
        calibration_on = bool(self.camera_calibration_enabled.get() and self.camera_calibration)
        alignment_on = bool(
            self.localization_alignment_enabled.get()
            and self.localization_alignment is not None
        )
        camera_matrix = None
        distortion = None
        if calibration_on:
            frame, camera_matrix = undistort_frame(frame, self.camera_calibration)
            distortion = np.zeros((5, 1), dtype=np.float32)
        self.current_camera_matrix = camera_matrix

        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        frame_sharp = cv2.filter2D(frame, -1, kernel)
        corners, detected_ids, _ = ARUCO_DETECTOR.detectMarkers(frame_sharp)
        primary_ids = (
            set() if detected_ids is None else set(detected_ids.flatten().tolist())
        )
        required_ids = set(ROBOT_IDS).union({4, 5, 6, 7})
        if not required_ids.issubset(primary_ids):
            fallback_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fallback_view = ARUCO_FALLBACK_CLAHE.apply(fallback_gray)
            fallback_corners, fallback_ids, _ = (
                ARUCO_FALLBACK_DETECTOR.detectMarkers(fallback_view)
            )
            corners, detected_ids = merge_aruco_detections(
                corners, detected_ids, fallback_corners, fallback_ids
            )
        display = frame.copy()
        ids = [] if detected_ids is None else detected_ids.flatten().tolist()
        current_raw_corners = {}
        measurements = {rid: {} for rid in ROBOT_IDS}
        W = float(self.real_width.get())
        H = float(self.real_height.get())

        with self.lock:
            if self.experiment_homography_mode.get() == "cruda" and not ids:
                self.homography = None
            elif (
                self.experiment_homography_mode.get() == "estabilizada"
                and self.homography is not None
                and (now - self.homography_t) > self.homography_hold_s
            ):
                self.homography = None

        if ids:
            cv2.aruco.drawDetectedMarkers(display, corners, np.array(ids))
            for i, mid in enumerate(ids):
                current_raw_corners[mid] = self._get_marker_center(corners[i][0])

            heavy_lock_threshold = 15.0
            ws_alpha = 0.8
            for mid in [4, 5, 6, 7]:
                if mid not in current_raw_corners:
                    continue
                raw_point = np.asarray(current_raw_corners[mid], dtype=np.float32)
                previous = self.ws_center_filt.get(mid)
                if previous is None:
                    self.ws_center_filt[mid] = raw_point
                else:
                    distance_px = float(np.linalg.norm(raw_point - previous))
                    # La camara es fija: promedia siempre el ruido pequeno y
                    # sigue lentamente un desplazamiento grande accidental.
                    keep_alpha = 0.98 if distance_px <= heavy_lock_threshold else ws_alpha
                    self.ws_center_filt[mid] = (
                        keep_alpha * previous + (1.0 - keep_alpha) * raw_point
                    )
                self.ws_last_seen[mid] = now

            homography_mode = self.experiment_homography_mode.get()
            if homography_mode == "cruda":
                draw_centers = current_raw_corners
            else:
                draw_centers = self.ws_center_filt
            if all(mid in draw_centers for mid in [4, 5, 6, 7]):
                polygon = np.array(
                    [draw_centers[mid] for mid in [4, 5, 6, 7]], np.int32
                ).reshape((-1, 1, 2))
                cv2.polylines(display, [polygon], True, (0, 255, 255), 3)

            world_points = np.array(
                [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], dtype=np.float32
            )
            homography_new = None
            if homography_mode == "cruda":
                if all(mid in current_raw_corners for mid in [4, 5, 6, 7]):
                    image_points = np.array(
                        [current_raw_corners[mid] for mid in [4, 5, 6, 7]], dtype=np.float32
                    )
                    homography_new = cv2.getPerspectiveTransform(image_points, world_points)
                with self.lock:
                    self.homography = homography_new
                    if homography_new is not None:
                        self.homography_t = now
            else:
                def recent(mid):
                    return mid in self.ws_center_filt and (
                        now - self.ws_last_seen.get(mid, 0.0)
                    ) <= self.homography_hold_s

                if all(recent(mid) for mid in [4, 5, 6, 7]):
                    image_points = np.array(
                        [self.ws_center_filt[mid] for mid in [4, 5, 6, 7]], dtype=np.float32
                    )
                    homography_new = cv2.getPerspectiveTransform(image_points, world_points)
                    with self.lock:
                        previous_h = None if self.homography is None else self.homography.copy()
                    if previous_h is not None:
                        normalized_new = homography_new / (homography_new[2, 2] + 1e-9)
                        normalized_previous = previous_h / (previous_h[2, 2] + 1e-9)
                        if float(np.linalg.norm(normalized_new - normalized_previous)) > 0.8:
                            homography_new = None
                with self.lock:
                    if homography_new is not None:
                        if self.homography is None:
                            self.homography = homography_new
                        else:
                            self.homography = 0.95 * self.homography + 0.05 * homography_new
                        self.homography_t = now

            with self.lock:
                homography_used = None if self.homography is None else self.homography.copy()

            if homography_used is not None:
                pose_centers = (
                    current_raw_corners
                    if homography_mode == "cruda"
                    else self.ws_center_filt
                )
                camera_pose_new = self._estimate_camera_pose_from_workspace(
                    ids,
                    corners,
                    W,
                    H,
                    frame.shape,
                    camera_matrix=camera_matrix,
                    distortion=distortion,
                    image_centers=pose_centers,
                )
                with self.lock:
                    if camera_pose_new is not None:
                        if self.cam_pos_world is None:
                            self.cam_pos_world = camera_pose_new
                        else:
                            previous_pose = np.asarray(self.cam_pos_world, dtype=np.float64)
                            candidate_pose = np.asarray(camera_pose_new, dtype=np.float64)
                            if float(np.linalg.norm(candidate_pose - previous_pose)) <= CAMERA_POSE_MAX_JUMP_M:
                                filtered_pose = (
                                    (1.0 - CAMERA_POSE_FILTER_ALPHA) * previous_pose
                                    + CAMERA_POSE_FILTER_ALPHA * candidate_pose
                                )
                                self.cam_pos_world = tuple(float(value) for value in filtered_pose)
                    camera_pose = self.cam_pos_world

                for i, mid in enumerate(ids):
                    if mid not in ROBOT_IDS:
                        continue
                    marker = corners[i][0]
                    center = self._get_marker_center(marker)
                    pixel_x, pixel_y = float(center[0]), float(center[1])
                    floor_p0 = self._transform_point(
                        homography_used, float(marker[0][0]), float(marker[0][1])
                    )
                    floor_p1 = self._transform_point(
                        homography_used, float(marker[1][0]), float(marker[1][1])
                    )
                    floor_x, floor_y = self._transform_point(homography_used, pixel_x, pixel_y)
                    corrected_p0 = floor_p0
                    corrected_p1 = floor_p1
                    corrected_x, corrected_y = floor_x, floor_y
                    if self.experiment_parallax_enabled.get():
                        marker_height = float(self.robot_marker_height_m.get())
                        corrected_p0 = self._parallax_correct_xy(*floor_p0, camera_pose, marker_height)
                        corrected_p1 = self._parallax_correct_xy(*floor_p1, camera_pose, marker_height)
                        corrected_x, corrected_y = self._parallax_correct_xy(
                            floor_x, floor_y, camera_pose, marker_height
                        )
                    aligned_x, aligned_y = apply_localization_alignment(
                        corrected_x,
                        corrected_y,
                        self.localization_alignment,
                        enabled=alignment_on,
                    )
                    raw_yaw = math.atan2(
                        corrected_p1[1] - corrected_p0[1],
                        corrected_p1[0] - corrected_p0[0],
                    )
                    measurement = {
                        "pixel_x": pixel_x,
                        "pixel_y": pixel_y,
                        "floor_x": floor_x,
                        "floor_y": floor_y,
                        "corrected_x": corrected_x,
                        "corrected_y": corrected_y,
                        "aligned_x": aligned_x,
                        "aligned_y": aligned_y,
                        "raw_yaw": raw_yaw,
                        "accepted": 0,
                    }
                    measurements[mid] = measurement

                    with self.lock:
                        previous_state = self.robot_state[mid]
                    if not self._valid_robot_pose(aligned_x, aligned_y, previous_state, now):
                        continue
                    if self.experiment_pose_filter_enabled.get():
                        final_x, final_y, final_yaw = self._filtered_robot_pose(
                            mid, aligned_x, aligned_y, raw_yaw, previous_state, now
                        )
                    else:
                        self.robot_pose_history[mid].clear()
                        final_x, final_y, final_yaw = aligned_x, aligned_y, raw_yaw
                    state = {
                        "x": final_x,
                        "y": final_y,
                        "yaw": final_yaw,
                        "t": now,
                        "frame_seq": frame_seq,
                        "frame_received_perf": frame_received_perf,
                        "processed_perf": time.perf_counter(),
                    }
                    pose_stages = {
                        "t": now,
                        "floor_x": floor_x,
                        "floor_y": floor_y,
                        "corrected_x": corrected_x,
                        "corrected_y": corrected_y,
                        "aligned_x": aligned_x,
                        "aligned_y": aligned_y,
                        "filtered_x": final_x,
                        "filtered_y": final_y,
                        "camera_pose": camera_pose,
                    }
                    with self.lock:
                        self.robot_state[mid] = state
                        self.latest_pose_stages[mid] = pose_stages
                    measurement["accepted"] = 1
                    cv2.putText(
                        display,
                        f"ID:{mid}",
                        (int(pixel_x), int(pixel_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )

        process_end = time.perf_counter()
        processing_ms = (process_end - process_start) * 1000.0
        with self.lock:
            homography_valid = self.homography is not None
            homography_age_ms = (
                max(0.0, (now - self.homography_t) * 1000.0) if homography_valid else float("nan")
            )
            camera_pose = self.cam_pos_world
            states_snapshot = {rid: self.robot_state[rid] for rid in ROBOT_IDS}
            for rid in ROBOT_IDS:
                info = self.discovered[rid]
                if info is not None and (time.time() - info["t"]) > ROBOT_FORGET_S:
                    self.discovered[rid] = None

        for rid in ROBOT_IDS:
            measurement = measurements[rid]
            state = states_snapshot[rid]
            pose_age_ms = (
                max(0.0, (now - state.get("t", now)) * 1000.0) if state else float("nan")
            )
            self.recorder.frame(
                frame_seq=frame_seq,
                frame_received_perf=frame_received_perf if frame_received_perf is not None else "",
                frame_wait_ms=(
                    max(0.0, (process_start - frame_received_perf) * 1000.0)
                    if frame_received_perf is not None else ""
                ),
                processing_ms=processing_ms,
                robot_id=rid,
                detected=1 if rid in ids else 0,
                accepted=measurement.get("accepted", 0),
                workspace_markers_seen=len(set(ids).intersection({4, 5, 6, 7})),
                homography_valid=1 if homography_valid else 0,
                homography_age_ms=homography_age_ms,
                calibration_enabled=1 if calibration_on else 0,
                homography_mode=self.experiment_homography_mode.get(),
                parallax_enabled=1 if self.experiment_parallax_enabled.get() else 0,
                alignment_enabled=1 if alignment_on else 0,
                pose_filter_enabled=1 if self.experiment_pose_filter_enabled.get() else 0,
                pixel_x=measurement.get("pixel_x", ""),
                pixel_y=measurement.get("pixel_y", ""),
                floor_x=measurement.get("floor_x", ""),
                floor_y=measurement.get("floor_y", ""),
                corrected_x=measurement.get("corrected_x", ""),
                corrected_y=measurement.get("corrected_y", ""),
                aligned_x=measurement.get("aligned_x", ""),
                aligned_y=measurement.get("aligned_y", ""),
                raw_yaw=measurement.get("raw_yaw", ""),
                x=state.get("x", "") if state else "",
                y=state.get("y", "") if state else "",
                yaw=state.get("yaw", "") if state else "",
                pose_age_ms=pose_age_ms,
                cam_x=camera_pose[0] if camera_pose else "",
                cam_y=camera_pose[1] if camera_pose else "",
                cam_z=camera_pose[2] if camera_pose else "",
            )
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
            frame_seq = self.latest_frame_seq
            frame_received_perf = self.latest_frame_received_perf
            disc = dict(self.discovered)
            camera_status = self.camera_status
            camera_error = self.camera_last_error

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
        self._update_grid_positioning()

        panel_status = camera_status
        if camera_error and camera_status != "Conectada":
            panel_status = f"{camera_status} | {camera_error[:80]}"
        self.panel_cam.config(text=f"Vista | Camara: {panel_status}")

        processed = None
        if frame is not None and frame_seq != self.last_processed_frame_seq:
            processed = self.process_frame(frame, frame_seq, frame_received_perf)
            self.last_processed_frame_seq = frame_seq
            self.latest_processed_display = processed
        elif self.latest_processed_display is not None:
            processed = self.latest_processed_display

        if processed is not None:
            rgb = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(rgb)
            pw, ph = self.panel_cam.winfo_width(), self.panel_cam.winfo_height()
            if pw > 100 and ph > 100:
                img_pil.thumbnail((pw, ph))
            imgtk = ImageTk.PhotoImage(image=img_pil)
            self.lbl_video.configure(image=imgtk, text="")
            self.lbl_video.image = imgtk
        else:
            self.lbl_video.configure(image="", text=panel_status)
            self.lbl_video.image = None

        self.draw_map()

        self.root.after(33, self._ui_loop)


def main():
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--auto-connect", action="store_true")
    parser.add_argument(
        "--pose-diagnostic",
        type=int,
        metavar="ROBOT_ID",
        help="Conecta sin mover y muestra las etapas de localizacion",
    )
    parser.add_argument(
        "--grid-position",
        nargs=3,
        type=int,
        metavar=("ROBOT_ID", "I", "J"),
        help="Conecta y posiciona de forma asistida en los indices de cuadricula",
    )
    parser.add_argument(
        "--grid-precision",
        type=float,
        metavar="METERS",
        help="Tolerancia de llegada usada con --grid-position",
    )
    parser.add_argument(
        "--run-paper-scenario",
        metavar="SCENARIO",
        help="Prepara, registra y ejecuta una repeticion fisica del paper",
    )
    parser.add_argument(
        "--controller-mode",
        choices=("apf_fsm", "apf_puro"),
        default="apf_fsm",
    )
    parser.add_argument("--condition", default="pilot")
    parser.add_argument("--replicate", type=int, default=1)
    parser.add_argument("--scenario-timeout", type=float, default=90.0)
    parser.add_argument("--safety-distance", type=float, default=0.12)
    parser.add_argument("--start-accuracy", type=float, default=0.035)
    parser.add_argument("--network-abort-timeout", type=float, default=30.0)
    parser.add_argument("--delay-ms", type=float, default=0.0)
    parser.add_argument("--jitter-ms", type=float, default=0.0)
    parser.add_argument(
        "--scenario-direction",
        choices=("auto", "forward", "reverse"),
        default="auto",
    )
    parser.add_argument(
        "--ack-probe-interval",
        type=float,
        help="Segundos entre sondas ACK; 0 desactiva ACK durante control",
    )
    args = parser.parse_args()

    root = tk.Tk()
    app = MultiRobotApp(root)

    if args.ack_probe_interval is not None:
        app.command_ack_probe_interval_s = max(0.0, float(args.ack_probe_interval))

    if args.grid_precision is not None:
        app.grid_accuracy_m.set(clamp(float(args.grid_precision), 0.005, 0.05))

    if args.run_paper_scenario:
        app.experiment_scenario.set(args.run_paper_scenario)
        app.experiment_controller_mode.set(args.controller_mode)
        app.experiment_condition.set(args.condition)
        app.experiment_replicate.set(max(1, int(args.replicate)))
        app.experiment_delay_ms.set(clamp(float(args.delay_ms), 0.0, 1000.0))
        app.experiment_jitter_ms.set(clamp(float(args.jitter_ms), 0.0, 500.0))
        app.experiment_homography_mode.set("estabilizada")
        app.camera_calibration_enabled.set(True)
        app.localization_alignment_enabled.set(True)
        app.experiment_parallax_enabled.set(True)
        app.experiment_pose_filter_enabled.set(True)
        app.avoid_on.set(True)
        app.avoid_radius.set(0.28)
        app.vmax_pct.set(35.0)
        app.experiment_collision_threshold_m.set(float(args.safety_distance))
        with app.lock:
            app.walls = []

    if (
        args.auto_connect
        or args.grid_position
        or args.pose_diagnostic
        or args.run_paper_scenario
    ):
        root.after(150, app.connect_camera)

    if args.grid_position:
        robot_id, index_i, index_j = args.grid_position
        app.selected_robot.set(robot_id)
        app.grid_i.set(index_i)
        app.grid_j.set(index_j)
        deadline = time.monotonic() + 20.0

        def start_when_ready():
            if robot_id not in ROBOT_IDS:
                app.experiment_status.set(f"Robot {robot_id} no configurado")
                return
            with app.lock:
                pose = app.robot_state.get(robot_id)
            _, network_status, _ = app.robot_net_status(robot_id)
            pose_ready = (
                pose is not None
                and time.time() - pose.get("t", 0.0) <= ROBOT_CONTROL_POSE_TIMEOUT_S
            )
            if pose_ready and network_status == "OK":
                app.position_selected_robot_at_grid()
                return
            if time.monotonic() >= deadline:
                app.experiment_status.set("Posicionamiento cancelado: vision o UDP no disponibles")
                return
            app.experiment_status.set("Esperando vision y UDP antes de posicionar...")
            root.after(300, start_when_ready)

        root.after(3000, start_when_ready)

    def on_close():
        app.stop_all()
        app.stop_experiment_recording("application_closed")
        app.running = False
        app.camera_wakeup.set()
        app.command_dispatcher.close()
        app.save_ui_config()
        try:
            app.cmd_sock.close()
            app.disc_sock.close()
        except OSError:
            pass
        root.destroy()
        import os
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)

    paper_automation = None
    if args.run_paper_scenario:
        def finish_paper_run(success, reason, run_dir):
            print(
                f"[PAPER_RUN] cierre success={int(success)} reason={reason} "
                f"run_dir={run_dir or '-'}",
                flush=True,
            )
            on_close()

        paper_automation = ScenarioRunAutomation(
            root=root,
            app=app,
            scenario_file=PAPER_SCENARIOS_FILE,
            scenario_name=args.run_paper_scenario,
            controller_mode=args.controller_mode,
            condition=args.condition,
            replicate=args.replicate,
            on_done=finish_paper_run,
            scenario_timeout_s=max(10.0, float(args.scenario_timeout)),
            start_accuracy_m=clamp(float(args.start_accuracy), 0.02, 0.10),
            safety_distance_m=clamp(float(args.safety_distance), 0.08, 0.30),
            network_abort_timeout_s=max(30.0, float(args.network_abort_timeout)),
            scenario_direction=args.scenario_direction,
        )
        root.after(1000, paper_automation.start)

    if args.pose_diagnostic:
        diagnostic_robot = int(args.pose_diagnostic)

        def emit_pose_diagnostic():
            with app.lock:
                homography = None if app.homography is None else app.homography.copy()
                frame_received_perf = app.latest_frame_received_perf
            workspace_centers = {
                mid: [float(value) for value in app.ws_center_filt[mid]]
                for mid in (4, 5, 6, 7)
                if mid in app.ws_center_filt
            }
            frame_age_ms = (
                None
                if frame_received_perf is None
                else (time.perf_counter() - frame_received_perf) * 1000.0
            )
            print(
                f"[POSE_DIAG] R{diagnostic_robot} "
                f"{app._grid_pose_summary(diagnostic_robot)}",
                flush=True,
            )
            print(
                "[POSE_DIAG] workspace_centers="
                + json.dumps(workspace_centers, separators=(",", ":")),
                flush=True,
            )
            print(
                "[POSE_DIAG] homography="
                + ("None" if homography is None else np.array2string(homography, precision=8))
                + f" frame_age_ms={frame_age_ms}",
                flush=True,
            )
            root.after(100, on_close)

        root.after(6000, emit_pose_diagnostic)

    root.mainloop()


if __name__ == "__main__":
    main()
