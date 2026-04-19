import threading
import time
import math
import socket
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import cv2
import numpy as np

# ============================
# CONFIG
# ============================
# Robots
ROBOT_IDS = [1, 2, 3]

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
DISCOVERY_INTERVAL_S = 0.7
ROBOT_STALE_S = 3.0  # si un robot no responde en X s se considera "perdido"

# Comandos UDP a robot
ROBOT_CMD_PORT = 44444  # todos usan este puerto (en el ESP32 también)
CMD_RATE_HZ = 12

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

# Umbrales más robustos (si se pierden marcadores)
ARUCO_PARAMS.adaptiveThreshWinSizeMin = 5
ARUCO_PARAMS.adaptiveThreshWinSizeMax = 45
ARUCO_PARAMS.adaptiveThreshWinSizeStep = 10
ARUCO_PARAMS.adaptiveThreshConstant = 7

ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# ============================
# CALIBRACIÓN DE CÁMARA (Manual)
# ============================

CAM_FX = None  # Ejemplo: 650.45
CAM_FY = None  # Ejemplo: 650.45
CAM_CX = None  # Ejemplo: 320.0
CAM_CY = None  # Ejemplo: 240.0
# Coeficientes de distorsión (k1, k2, p1, p2, k3)
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
        self.root.geometry("1400x780")

        self.running = True
        self.lock = threading.Lock()

        # ---------- Video ----------
        self.cap = None
        self.latest_frame = None
        self.url_camera = tk.StringVar(value="http://172.25.203.138:5000/video") 

        # ---------- Workspace ----------
        self.real_width = tk.DoubleVar(value=1.25)
        self.real_height = tk.DoubleVar(value=1.25)
        self.homography = None

        # --- Estabilidad homografía ---
        self.homography_t = 0.0  # cuándo se actualizó por última vez
        self.homography_hold_s = 1.2  # segundos que “aguanta” el último H válido
        self.ws_center_filt = {}  # centros filtrados de IDs 4..7  (id -> np.array([x,y]))
        self.ws_last_seen = {}  # último tiempo visto por ID

        # --- Parallax / altura ---
        self.robot_marker_height_m = tk.DoubleVar(value=0.06)  # 6 cm
        self.cam_pos_world = None  # (Cx, Cy, Cz) en metros, en coords del mundo

        # ---------- Control ----------
        self.control_active = tk.BooleanVar(value=False)
        self.selected_robot = tk.IntVar(value=1)

        # Ganancias (en %)
        # Klin: Velocidad lineal
        self.k_lin_pct_per_m = tk.DoubleVar(value=70.0)  # % por metro
        # Kang: Velocidad de giro.
        self.k_ang_pct_per_rad = tk.DoubleVar(value=10.0)  # % por rad
        # Vmax: Velocidad tope.
        self.vmax_pct = tk.DoubleVar(value=40.0)
        self.wspin_thresh_rad = tk.DoubleVar(value=0.55)  # ~31°
        self.dist_tolerance = tk.DoubleVar(value=0.02)  # 2 cm

        # Memoria para el control Derivativo (D)
        self.prev_angle_err = {rid: 0.0 for rid in ROBOT_IDS}
        self.k_ang_d_pct = tk.DoubleVar(value=3.5)

        # Evitación
        self.avoid_on = tk.BooleanVar(value=True)
        self.avoid_radius = tk.DoubleVar(value=0.20)  
        self.k_rep = tk.DoubleVar(value=0.70)  

        # ---------- Estado robots (visión) ----------
        # robot_state[rid] = {"x":, "y":, "yaw":, "t":}
        self.robot_state = {rid: None for rid in ROBOT_IDS}

        # ---------- Objetivos ----------
        # target[rid] = (x,y) o None
        self.targets = {rid: None for rid in ROBOT_IDS}

        # ---------- Red ----------
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Discovery: escucha respuestas
        self.disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.disc_sock.bind(("", DISCOVERY_PORT))
        self.disc_sock.settimeout(0.2)

        # Tabla de IPs descubiertas
        # discovered[rid] = {"ip": str, "port": int, "t": float}
        self.discovered = {rid: None for rid in ROBOT_IDS}

        # === VISUALIZACIÓN DE FUERZAS ===
        # Guardaremos aquí los vectores calculados para dibujarlos luego
        self.vis_vectors = {rid: {'att': None, 'rep': None, 'res': None} for rid in ROBOT_IDS}

        # ---------- UI ----------
        self._setup_ui()

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

        tk.Label(top, text=" | Robot activo:", bg="#ddd").pack(side=tk.LEFT)
        ttk.Combobox(top, textvariable=self.selected_robot, values=ROBOT_IDS, width=4, state="readonly").pack(
            side=tk.LEFT)

        tk.Checkbutton(top, text="Evitar choques", variable=self.avoid_on, bg="#ddd").pack(side=tk.LEFT, padx=8)
        tk.Label(top, text="R(m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.avoid_radius, width=5).pack(side=tk.LEFT)

        # Ganancias
        tk.Label(top, text=" | Klin(%/m):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.k_lin_pct_per_m, width=6).pack(side=tk.LEFT)
        tk.Label(top, text="Kang(%/rad):", bg="#ddd").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.k_ang_pct_per_rad, width=6).pack(side=tk.LEFT)

        # Estado discovery
        self.lbl_net = tk.Label(top, text="Discovery: ...", bg="#ddd")
        self.lbl_net.pack(side=tk.RIGHT, padx=10)

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
        self.control_active.set(False)
        with self.lock:
            for rid in ROBOT_IDS:
                self.targets[rid] = None
        for rid in ROBOT_IDS:
            self.send_robot_cmd(rid, 0, 0)

    def on_map_right_click(self, event):
        rid = int(self.selected_robot.get())
        tx, ty = self.map_to_world(event.x, event.y)
        if tx is None:
            return
        with self.lock:
            self.targets[rid] = (tx, ty)

    def on_map_middle_click(self, event):
        rid = int(self.selected_robot.get())
        with self.lock:
            self.targets[rid] = None
        self.send_robot_cmd(rid, 0, 0)

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
    def _discovery_loop(self):
        while self.running:
            try:
                # 1) broadcast query
                self.disc_sock.sendto(DISCOVERY_QUERY, ("255.255.255.255", DISCOVERY_PORT))

                # 2) leer respuestas un ratito
                t_end = time.time() + 0.25
                while time.time() < t_end:
                    try:
                        data, addr = self.disc_sock.recvfrom(256)
                    except socket.timeout:
                        break

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
                            with self.lock:
                                self.discovered[rid] = {"ip": addr[0], "port": port, "t": time.time()}

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
            return  # no descubierto aún

        ip = info["ip"]
        port = info["port"]
        msg = f"M {left_pct} {right_pct}".encode()
        try:
            self.cmd_sock.sendto(msg, (ip, port))
        except Exception:
            pass

    # =========================
    # CONTROL THREAD (3 robots)
    # =========================
    def _control_loop(self):
        dt = 1.0 / CMD_RATE_HZ

        # === Máquina de estados por robot (reposo real / orientar / correr / evasión)
        # IDLE  : reposo real (sin target o recién llegó)
        # ORIENT: solo gira hasta quedar dentro de ±10°
        # RUN   : navegación normal
        # AVOID : evasión por repulsión (al salir vuelve a RUN, no a ORIENT)
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

            for rid in ROBOT_IDS:
                st = states[rid]
                goal = targets[rid]

                # --- Estado base por visión ---
                if st is None:
                    self.send_robot_cmd(rid, 0, 0)
                    continue

                # --- Reposo real (SIN objetivo) ---
                if goal is None:
                    self.send_robot_cmd(rid, 0, 0)
                    self.nav_mode[rid] = "IDLE"
                    self.prev_goal[rid] = None
                    self.prev_angle_err[rid] = 0.0
                    continue

                # Si venimos de reposo real (IDLE) y ahora hay objetivo -> primero orientar
                if self.nav_mode.get(rid, "IDLE") == "IDLE":
                    self.nav_mode[rid] = "ORIENT"
                    self.prev_angle_err[rid] = 0.0

                # Guardar el objetivo actual (para distinguir reposo real vs cambio dinámico)
                self.prev_goal[rid] = goal

                rx, ry, yaw = st["x"], st["y"], st["yaw"]
                gx, gy = goal

                # --- 1. LLEGADA ---
                dist_goal = math.hypot(gx - rx, gy - ry)
                if dist_goal < float(self.dist_tolerance.get()):
                    self.send_robot_cmd(rid, 0, 0)
                    with self.lock:
                        self.targets[rid] = None

                    # Reposo real
                    self.nav_mode[rid] = "IDLE"
                    self.prev_goal[rid] = None
                    self.prev_angle_err[rid] = 0.0
                    continue

                # --- 2. ATRACCIÓN ---
                dist_vector = np.array([gx - rx, gy - ry], dtype=np.float32)
                norm_goal = float(np.linalg.norm(dist_vector))
                if norm_goal > 1e-6:
                    u_goal = dist_vector / norm_goal
                else:
                    u_goal = np.array([0.0, 0.0], dtype=np.float32)

                # --- 3. REPULSIÓN (Simple, sin tangencial) ---
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

                # Limitar repulsión
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

                # Heading hacia la resultante (objetivo + repulsión) para RUN/AVOID
                desired_heading_res = math.atan2(float(u[1]), float(u[0]))

                mode = self.nav_mode.get(rid, "IDLE")

                # --- 5.1 Cambiar a modo evasión si hay repulsión relevante ---
                IS_SAFE_ZONE = (norm_rep < 0.15)  # tu criterio actual
                if self.avoid_on.get() and (not IS_SAFE_ZONE):
                    if mode != "AVOID":
                        self.prev_angle_err[rid] = 0.0
                    mode = "AVOID"
                else:
                    # Si estábamos evitando y ya salimos, volvemos a RUN (NO a ORIENT)
                    if mode == "AVOID":
                        self.prev_angle_err[rid] = 0.0
                        mode = "RUN"

                # --- 5.2 Elegir heading según modo ---
                if mode == "ORIENT":
                    desired_heading = desired_heading_goal
                else:
                    desired_heading = desired_heading_res

                angle_err = wrap_pi(desired_heading - yaw)

                # Leer Ganancias
                klin = float(self.k_lin_pct_per_m.get())
                kp_ang = float(self.k_ang_pct_per_rad.get())
                kd_ang = float(self.k_ang_d_pct.get())
                vmax = float(self.vmax_pct.get())

                # Control PD Angular
                prev_err = self.prev_angle_err.get(rid, 0.0)
                d_err = angle_err - prev_err
                self.prev_angle_err[rid] = angle_err

                angular_raw = (kp_ang * angle_err) + (kd_ang * d_err * 10.0)
                angular_val = clamp(angular_raw, -vmax, vmax)

                # === AJUSTE: velocidad de giro SOLO en ORIENT (reposo real) ===
                # Aumenta o disminuye la rapidez con la que gira mientras está "alineándose" en reposo real.
                mode_now = self.nav_mode.get(rid, "IDLE")

                ORIENT_TURN_GAIN = 1.4  # subir para girar más rápido (ej: 2.5)
                ORIENT_MIN_TURN = 10.0  # mínimo de giro (%) para que no se quede "temblando" 

                if mode_now == "ORIENT":
                    # Escalar la orden angular
                    angular_val *= ORIENT_TURN_GAIN

                    # Asegurar un mínimo de giro para vencer fricción / zona muerta
                    if abs(angular_val) < ORIENT_MIN_TURN and abs(angle_err) > math.radians(2.0):
                        angular_val = math.copysign(ORIENT_MIN_TURN, angular_val if angular_val != 0 else angle_err)

                    # Re-limitar por seguridad
                    angular_val = clamp(angular_val, -vmax, vmax)

                # 1. Throttle por distancia (igual que antes)
                dist_factor = min(dist_goal / 0.15, 1.0)

                # 2. Márgenes (en rad)
                MARGIN_ORIENT = math.radians(15.0)  # ±10°
                MARGIN_RUN = math.radians(286.0)  # "286°"
                MARGIN_RUN = min(MARGIN_RUN, math.pi)

                # 3. Lógica por estados
                if mode == "ORIENT":
                    # En ORIENT: no avanza, solo gira hasta quedar dentro de ±10°
                    align_factor = 0.0
                    dist_factor = 0.0  # fuerza lineal=0

                    if abs(angle_err) <= MARGIN_ORIENT:
                        mode = "RUN"
                        self.prev_angle_err[rid] = 0.0

                elif mode == "RUN":
                    # En RUN: aplica tu lógica normal en zona segura
                    if IS_SAFE_ZONE:
                        align_factor = 1.0 if abs(angle_err) <= MARGIN_RUN else 0.0
                    else:
                        # Si por algún motivo estamos RUN pero aparece repulsión,
                
                        align_factor = max(0.0, math.cos(angle_err))

                else:  # AVOID
                    # Evasión: mantenemos coseno 
                    align_factor = max(0.0, math.cos(angle_err))

                # Guardar modo final
                self.nav_mode[rid] = mode

                # Si align_factor es bajo (robot frenado o curveando cerrado),
                if mode != "ORIENT" and align_factor < 0.5:
                    # Interpolación Lineal Inversa:
                    # - Si align_factor es 0.0 (Parado) -> Boost = 3.5 (Giro muy rápido)
                    # - Si align_factor es 0.4 (Curva)  -> Boost = 1.5 (Giro alegre)
                    # - Si align_factor es 0.5 (Recto)  -> Boost = 1.0 (Normal)

                    boost = 2.5 - (align_factor * 3.0)
                    boost = max(1.0, boost)  # Nunca bajar de 1.0

                    angular_val *= boost
                    # Re-limitamos para no saturar
                    angular_val = clamp(angular_val, -vmax, vmax)

                # Calculamos velocidad lineal final
                raw_linear = klin * dist_factor * align_factor
                linear_val = clamp(raw_linear, 0, vmax)

                # Mezclamos lineal y angular SIN usar "if error > spin_th"
                left = linear_val - angular_val
                right = linear_val + angular_val

                # Zona muerta y Clamping
                left = int(clamp(left, -100, 100))
                right = int(clamp(right, -100, 100))

                MIN_PWM = 12
                if abs(left) < MIN_PWM and abs(left) > 1: left = math.copysign(MIN_PWM, left)
                if abs(right) < MIN_PWM and abs(right) > 1: right = math.copysign(MIN_PWM, right)

                self.send_robot_cmd(rid, left, right)

            time.sleep(dt)

    # =========================
    # VISION: detección + homografía
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

        # 2) Matriz intrínseca (K) y Distorsión (dist)
        if None not in (CAM_FX, CAM_FY, CAM_CX, CAM_CY, CAM_DIST):
            # Usar valores reales calibrados
            K = np.array([[CAM_FX, 0, CAM_CX],
                          [0, CAM_FY, CAM_CY],
                          [0, 0, 1]], dtype=np.float32)
            dist = np.array(CAM_DIST, dtype=np.float32)
        else:
            # Usar aproximación (fallback)
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
        C = (-R.T @ tvec).reshape(-1)  # cámara en coords del mundo
        if C[2] < 0.40:
            return None
        return (float(C[0]), float(C[1]), float(C[2]))

    def _parallax_correct_xy(self, x_floor, y_floor, cam_pos, h_obj):
        """
        Dado el punto que te da la homografía (intersección con suelo z=0),
        corrige para obtener el XY del objeto a altura h_obj (m) sobre el suelo.

        Fórmula: P_h = Cxy + ((Cz - h)/Cz) * (P0 - Cxy)
        """
        if cam_pos is None:
            return x_floor, y_floor

        cx, cy, cz = cam_pos
        if cz <= (h_obj + 0.1):
            return x_floor, y_floor  # evita división rara

        s = (cz - h_obj) / cz  # < 1  (trae el punto hacia la cámara)

        # === LIMITADOR DE EXPLOSIÓN ===
        # Si la corrección intenta mover el punto más de un 200% relativo al centro, lo ignoramos
        if abs(s) > 2.0:
            return x_floor, y_floor

        x = cx + (x_floor - cx) * s
        y = cy + (y_floor - cy) * s
        return x, y


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

        # Diccionario para guardar dónde están las esquinas RAW (crudas) en este frame
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

            # Umbral alto: El marcador debe moverse más de X px para ser actualizado.
            HEAVY_LOCK_THRESHOLD = 15.0
            WS_ALPHA = 0.8  # Velocidad de actualización 

            for mid in [4, 5, 6, 7]:
                if mid in current_raw_corners:
                    raw_p = np.array(current_raw_corners[mid], dtype=np.float32)

                    prev = self.ws_center_filt.get(mid)

                    if prev is None:
                        # Primera vez que lo vemos: guardar directo
                        self.ws_center_filt[mid] = raw_p
                        self.ws_last_seen[mid] = now
                    else:
                        # Ya lo conocíamos. Calculamos cuánto se movió respecto al ANCLA.
                        dist_moved = np.linalg.norm(raw_p - prev)

                        if dist_moved > HEAVY_LOCK_THRESHOLD:
                            # CAMBIO INTENCIONAL: El usuario movió el marcador lejos.
                            # Actualizamos el filtro (suavemente para no saltar de golpe)
                            self.ws_center_filt[mid] = (WS_ALPHA * prev) + ((1.0 - WS_ALPHA) * raw_p)
                            self.ws_last_seen[mid] = now
                        else:
                            # RUIDO / VIBRACIÓN: El marcador se movió poco (ej. 4px).
                            # IGNORAMOS la nueva lectura. Mantenemos 'prev' inmutable.
                            # Solo actualizamos el tiempo 'last_seen' para saber que sigue vivo.
                            self.ws_last_seen[mid] = now

            # ==================================================================
            # 3. DIBUJAR LÍNEAS AMARILLAS USANDO LOS DATOS FILTRADOS (ESTABLES)
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
            # 4. CALCULO DE HOMOGRAFÍA (Usando los centros filtrados)
            # ==================================================================

            W = float(self.real_width.get())
            H = float(self.real_height.get())

            # Verificar si todos los marcadores 4..7 han sido vistos recientemente
            # (aunque no estén en este frame exacto, usamos su memoria)
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

            # Actualizar homografía global
            with self.lock:
                if Hm_new is not None:
                    if self.homography is None:
                        self.homography = Hm_new
                    else:
                        H_ALPHA = 0.95
                        self.homography = (H_ALPHA * self.homography) + ((1.0 - H_ALPHA) * Hm_new)
                    self.homography_t = now

            # ==================================================================
            # 5. ROBOTS (IDS 1, 2, 3)
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

                        final_x, final_y, final_yaw = rx_raw, ry_raw, yaw_raw

                        if prev_st is not None:
                            prev_x, prev_y = prev_st["x"], prev_st["y"]
                            prev_yaw = prev_st["yaw"]

                            # A) Zona Muerta: Si se movió menos de 5mm, es ruido -> MANTENER ANTERIOR
                            dist_moved = math.hypot(rx_raw - prev_x, ry_raw - prev_y)
                            DEADZONE_M = 0.005  # 5 milímetros

                            if dist_moved < DEADZONE_M:
                                final_x = prev_x
                                final_y = prev_y
                                # Mantenemos el yaw anterior también para evitar giros fantasma
                                final_yaw = prev_yaw
                            else:
                                # B) Filtro Suavizado (EMA): Si se movió, interpolamos
                                # alpha bajo = mucho suavizado (lento), alpha alto = poco suavizado (rápido)
                                ROBOT_ALPHA = 0.4  # 40% dato nuevo, 60% historia

                                final_x = (prev_x * (1.0 - ROBOT_ALPHA)) + (rx_raw * ROBOT_ALPHA)
                                final_y = (prev_y * (1.0 - ROBOT_ALPHA)) + (ry_raw * ROBOT_ALPHA)

                                # Interpolación angular correcta (evita el problema de -pi a pi)
                                diff = wrap_pi(yaw_raw - prev_yaw)
                                final_yaw = wrap_pi(prev_yaw + (diff * ROBOT_ALPHA))

                        # 3. Guardar estado final
                        with self.lock:
                            self.robot_state[mid] = {"x": final_x, "y": final_y, "yaw": final_yaw, "t": now}

                        # Overlay Robot (Texto con pos suavizada)
                        cv2.putText(display, f"ID:{mid}",
                                    (int(cx), int(cy) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Limpieza de viejos
        with self.lock:
            # Discovery cleanup
            for rid in ROBOT_IDS:
                info = self.discovered[rid]
                if info is not None and (time.time() - info["t"]) > ROBOT_STALE_S:
                    self.discovered[rid] = None

        return display

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

            # Ajustamos un poquito el texto para que no quede encima de la línea
            # Si es la parte de abajo (cy < H/2), texto más abajo (+15)
            # Si es la parte de arriba, texto más arriba (-15)
            offset_y = 15 if cy < H / 2 else -15

            self.canvas.create_text(cmx, cmy + offset_y, text=f"ID {cid}", fill="blue", font=("Arial", 10, "bold"))

        # Targets
        with self.lock:
            targets = dict(self.targets)
            states = dict(self.robot_state)
            discovered = dict(self.discovered)

        for rid, goal in targets.items():
            if goal is None:
                continue
            gx, gy = goal
            mx, my, _, _, _ = self.world_to_map(gx, gy, cw, ch, W, H)  # Usa la nueva función
            self.canvas.create_oval(mx - 6, my - 6, mx + 6, my + 6, fill="red", outline="")
            self.canvas.create_text(mx, my - 14, text=f"G{rid}", fill="red")

        # Robots
        for rid, st in states.items():
            if st is None:
                continue
            rx, ry, yaw = st["x"], st["y"], st["yaw"]

            # Convertir a pixeles con la Y invertida
            mx, my, _, _, _ = self.world_to_map(rx, ry, cw, ch, W, H)

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
                self.canvas.create_text(mx, my - 18, text=info["ip"], fill="gray25")

            # === DIBUJAR FUERZAS Y PAREDES ===
            # 1. Dibujar Zona de Paredes (Rectángulo Rojo Tenue)
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

                # Posición del robot en pixeles
                mx, my, _, _, _ = self.world_to_map(st["x"], st["y"], cw, ch, W, H)

                # Dibujar Radio de Evasión (Círculo punteado)
                r_pix = float(self.avoid_radius.get()) * scale
                self.canvas.create_oval(mx - r_pix, my - r_pix, mx + r_pix, my + r_pix,
                                        outline="#FFA500", dash=(2, 2))

                # Dibujar Flechas (Atracción, Repulsión, Resultante)
                # Nota: En pantalla Y crece hacia abajo, en matemáticas hacia arriba.
                # Por eso restamos vector_y (my - vy).

                if vecs['att'] is not None:
                    # Atracción (VERDE)
                    vx, vy = vecs['att']
                    self.canvas.create_line(mx, my, mx + vx * VIS_SCALE, my - vy * VIS_SCALE,
                                            fill="green", width=2, arrow=tk.LAST)

                if vecs['rep'] is not None:
                    # Repulsión (ROJO)
                    vx, vy = vecs['rep']
                    # Solo dibujamos si hay repulsión significativa
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
        net_txt = " | ".join([f"R{rid}:{disc[rid]['ip'] if disc[rid] else '---'}" for rid in ROBOT_IDS])
        self.lbl_net.config(text=f"Discovery: {net_txt}")

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
