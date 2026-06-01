import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

# ============================
# CONFIGURACIÓN DE RED (IGUAL QUE TU SISTEMA ACTUAL)
# ============================
DISCOVERY_PORT = 37030
CMD_PORT = 44444
DISCOVERY_MSG = b"DISCOVER_ROBOTS"


class RobotDebugger:
    def __init__(self, root):
        self.root = root
        self.root.title("Herramienta de Diagnóstico de Motores UDP")
        self.root.geometry("600x500")

        self.target_ip = None
        self.running = True

        # Comandos actuales
        self.cmd_l = 0
        self.cmd_r = 0
        self.status_msg = "Buscando robot..."

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", DISCOVERY_PORT))
        self.sock.settimeout(0.2)

        # UI
        self.setup_ui()

        # Hilos
        self.th_disc = threading.Thread(target=self.discovery_loop, daemon=True)
        self.th_disc.start()

        self.th_send = threading.Thread(target=self.sender_loop, daemon=True)
        self.th_send.start()

        self.update_ui()

    def setup_ui(self):
        # Info IP
        self.lbl_info = tk.Label(self.root, text="Esperando robot...", font=("Arial", 12, "bold"), fg="blue")
        self.lbl_info.pack(pady=10)

        # Panel de pruebas
        frame_controls = tk.LabelFrame(self.root, text="Pruebas de Movimiento (Mantener presionado)")
        frame_controls.pack(pady=10, padx=10, fill="both", expand=True)

        # Botones grandes
        btn_fwd = tk.Button(frame_controls, text="ADELANTE (Ambos +40)", bg="#ccffcc")
        btn_fwd.bind('<ButtonPress-1>', lambda e: self.set_cmd(40, 40))
        btn_fwd.bind('<ButtonRelease-1>', lambda e: self.stop())
        btn_fwd.pack(pady=5, fill="x", padx=20)

        btn_left = tk.Button(frame_controls, text="GIRO IZQUIERDA (Izq -40, Der +40)", bg="#ffffcc")
        btn_left.bind('<ButtonPress-1>', lambda e: self.set_cmd(-40, 40))
        btn_left.bind('<ButtonRelease-1>', lambda e: self.stop())
        btn_left.pack(pady=5, fill="x", padx=20)

        btn_right = tk.Button(frame_controls, text="GIRO DERECHA (Izq +40, Der -40)", bg="#ffffcc")
        btn_right.bind('<ButtonPress-1>', lambda e: self.set_cmd(40, -40))
        btn_right.bind('<ButtonRelease-1>', lambda e: self.stop())
        btn_right.pack(pady=5, fill="x", padx=20)

        tk.Label(frame_controls, text="--- DIAGNÓSTICO DE CABLES ---", fg="gray").pack(pady=10)

        btn_only_l = tk.Button(frame_controls, text="SOLO MOTOR IZQUIERDO (+40, 0)", bg="#e6e6fa")
        btn_only_l.bind('<ButtonPress-1>', lambda e: self.set_cmd(40, 0))
        btn_only_l.bind('<ButtonRelease-1>', lambda e: self.stop())
        btn_only_l.pack(pady=5, fill="x", padx=20)

        btn_only_r = tk.Button(frame_controls, text="SOLO MOTOR DERECHO (0, +40)", bg="#e6e6fa")
        btn_only_r.bind('<ButtonPress-1>', lambda e: self.set_cmd(0, 40))
        btn_only_r.bind('<ButtonRelease-1>', lambda e: self.stop())
        btn_only_r.pack(pady=5, fill="x", padx=20)

        self.lbl_debug = tk.Label(self.root, text="Enviando: M 0 0", font=("Consolas", 12), bg="black", fg="#00ff00")
        self.lbl_debug.pack(side="bottom", fill="x")

    def set_cmd(self, l, r):
        self.cmd_l = l
        self.cmd_r = r
        self.lbl_debug.config(text=f"Enviando: M {l} {r}")

    def stop(self):
        self.cmd_l = 0
        self.cmd_r = 0
        self.lbl_debug.config(text=f"Enviando: M 0 0 (STOP)")

    def discovery_loop(self):
        while self.running:
            try:
                self.sock.sendto(DISCOVERY_MSG, ("255.255.255.255", DISCOVERY_PORT))
                try:
                    data, addr = self.sock.recvfrom(1024)
                    msg = data.decode()
                    if "ROBOT_HERE" in msg:
                        if self.target_ip != addr[0]:
                            self.target_ip = addr[0]
                            self.status_msg = f"Robot detectado en: {self.target_ip}"
                except socket.timeout:
                    pass
            except Exception as e:
                print(f"Error Discovery: {e}")
            time.sleep(1)

    def sender_loop(self):
        while self.running:
            if self.target_ip:
                msg = f"M {self.cmd_l} {self.cmd_r}".encode()
                try:
                    self.sock.sendto(msg, (self.target_ip, CMD_PORT))
                except:
                    pass
            # Enviar a 20Hz para mantener vivo el robot
            time.sleep(0.05)

    def update_ui(self):
        self.lbl_info.config(text=self.status_msg)
        self.root.after(100, self.update_ui)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDebugger(root)
    root.mainloop()