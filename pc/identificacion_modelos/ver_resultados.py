import argparse
import csv
import json
import os
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

import numpy as np

try:
    from .analysis import first_order_delay_response, plot_results
except ImportError:
    from analysis import first_order_delay_response, plot_results


BASE_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BASE_DIR / "resultados"


def latest_run(robot_id):
    robot_dir = RESULTS_DIR / f"robot_{robot_id}"
    runs = [p for p in robot_dir.iterdir() if p.is_dir()] if robot_dir.exists() else []
    if not runs:
        raise FileNotFoundError(f"No hay corridas en {robot_dir}")
    return max(runs, key=lambda p: p.stat().st_mtime)


def available_runs(robot_id):
    robot_dir = RESULTS_DIR / f"robot_{robot_id}"
    if not robot_dir.exists():
        return []
    return sorted([p for p in robot_dir.iterdir() if p.is_dir()], key=lambda p: p.stat().st_mtime, reverse=True)


def load_json(path, default):
    if not path.exists():
        return default
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_samples(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    samples = []
    for row in rows:
        converted = {}
        for key, value in row.items():
            if key == "mode":
                converted[key] = value
            else:
                try:
                    converted[key] = float(value)
                except (TypeError, ValueError):
                    converted[key] = value
        samples.append(converted)
    return samples


def load_result_bundle(run_dir):
    run_dir = Path(run_dir).resolve()
    return {
        "run_dir": run_dir,
        "frequency_response": load_json(run_dir / "frequency_response.json", {}),
        "model": load_json(run_dir / "model.json", {}),
        "diff_model": load_json(run_dir / "modelo_diferencial.json", {}),
        "pid": load_json(run_dir / "pid_params.json", {}),
        "summary": load_json(run_dir / "experiment_summary.json", {}),
        "samples": load_samples(run_dir / "samples.csv"),
    }


def model_equation(mode, model):
    label = "Gv" if mode == "lineal" else "Gw"
    output = "v(s)" if mode == "lineal" else "w(s)"
    input_name = "u_v(s)" if mode == "lineal" else "u_w(s)"
    if not model:
        return f"{label}(s): sin modelo ajustado"
    k = model.get("gain", 0.0)
    tau = model.get("tau_s", 0.0)
    delay = model.get("delay_s", 0.0)
    return (
        f"{output}/{input_name} = {label}(s) = "
        f"{k:.6g} / ({tau:.4f}s + 1) * exp(-{delay:.4f}s)"
    )


def build_summary_text(bundle):
    run_dir = bundle["run_dir"]
    model = bundle["model"]
    pid = bundle["pid"]
    summary = bundle["summary"]
    freq = bundle["frequency_response"]
    samples = bundle["samples"]

    lines = []
    lines.append(f"Corrida: {run_dir}")
    lines.append(f"Muestras: {summary.get('n_samples', len(samples))}")
    lines.append("")
    lines.append("Modelo diferencial del robot")
    lines.append("u_v = (left + right) / 2")
    lines.append("u_w = (right - left) / 2")
    lines.append("[v]   [Gv(s)   0   ] [u_v]")
    lines.append("[w] = [  0   Gw(s)] [u_w]")
    lines.append("")

    for mode in ["lineal", "angular"]:
        lines.append(model_equation(mode, model.get(mode)))
        m = model.get(mode)
        if m:
            lines.append(
                f"  K={m.get('gain', 0):.6g}, tau={m.get('tau_s', 0):.4f}s, "
                f"delay={m.get('delay_s', 0):.4f}s, mse={m.get('fit_mse', 0):.4g}, "
                f"metodo={m.get('fit_method', '---')}"
            )
        lines.append(f"  puntos Bode: {len(freq.get(mode, []))}")
        lines.append("")

    incomplete = summary.get("incomplete_frequencies", [])
    if incomplete:
        lines.append("Frecuencias incompletas:")
        for item in incomplete:
            lines.append(f"  {item.get('mode')} {item.get('freq_hz')} Hz: {item.get('reason')}")
        lines.append("")

    lines.append("Controlador calculado")
    for section in ["lineal", "angular"]:
        data = pid.get(section)
        if not data:
            lines.append(f"{section}: sin PID calculado")
            continue
        lines.append(f"{section}:")
        for key, value in data.items():
            lines.append(f"  {key}: {value}")

    compat = pid.get("compat_pc_servidor_vision", {})
    if compat:
        lines.append("")
        lines.append("Valores compatibles con pc_servidor_vision.py:")
        for key, value in compat.items():
            lines.append(f"  {key}: {value}")

    lines.append("")
    lines.append("Archivos esperados:")
    for name in [
        "samples.csv",
        "frequency_response.json",
        "model.json",
        "modelo_diferencial.json",
        "pid_params.json",
        "bode_lineal.png",
        "bode_angular.png",
        "timeseries.png",
    ]:
        status = "OK" if (run_dir / name).exists() else "---"
        lines.append(f"  {status} {name}")

    return "\n".join(lines)


def print_summary(bundle):
    print(build_summary_text(bundle))


def show_images_system(run_dir):
    images = [p for p in [
        run_dir / "bode_lineal.png",
        run_dir / "bode_angular.png",
        run_dir / "timeseries.png",
    ] if p.exists()]

    if not images:
        print("No hay PNG para mostrar.")
        return

    for image_path in images:
        print(f"Abriendo {image_path}")
        try:
            os.startfile(str(image_path))
        except AttributeError:
            pass


class ResultViewerApp:
    def __init__(self, root, initial_run=None, initial_robot=1):
        self.root = root
        self.root.title("Resultados de identificacion de modelos")
        self.root.geometry("1280x820")
        self.robot_var = tk.IntVar(value=initial_robot)
        self.run_var = tk.StringVar()
        self.current_run = None
        self.bundle = None
        self.figures = []

        self._setup_ui()
        self.refresh_runs()
        if initial_run:
            self.load_run(Path(initial_run))
        else:
            try:
                self.load_run(latest_run(initial_robot))
            except FileNotFoundError:
                self.set_summary_text("No hay resultados para mostrar.")

    def _setup_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Robot:").pack(side=tk.LEFT)
        robot_box = ttk.Combobox(top, textvariable=self.robot_var, values=[1, 2, 3], width=5, state="readonly")
        robot_box.pack(side=tk.LEFT, padx=4)
        robot_box.bind("<<ComboboxSelected>>", lambda _e: self.refresh_runs(load_latest=True))

        ttk.Label(top, text="Corrida:").pack(side=tk.LEFT, padx=(12, 2))
        self.run_box = ttk.Combobox(top, textvariable=self.run_var, width=28, state="readonly")
        self.run_box.pack(side=tk.LEFT, padx=4)
        self.run_box.bind("<<ComboboxSelected>>", lambda _e: self.load_selected_run())

        ttk.Button(top, text="Actualizar", command=lambda: self.refresh_runs(load_latest=True)).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Elegir carpeta", command=self.choose_folder).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Regenerar PNG", command=self.regenerate_png).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Abrir carpeta", command=self.open_folder).pack(side=tk.LEFT, padx=4)

        self.path_label = ttk.Label(self.root, text="", anchor="w")
        self.path_label.pack(side=tk.TOP, fill=tk.X, padx=8)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.summary_text = scrolledtext.ScrolledText(self.notebook, wrap=tk.WORD, font=("Consolas", 10))
        self.notebook.add(self.summary_text, text="Resumen / modelo")

        self.bode_linear_frame = ttk.Frame(self.notebook)
        self.bode_angular_frame = ttk.Frame(self.notebook)
        self.timeseries_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.bode_linear_frame, text="Bode lineal")
        self.notebook.add(self.bode_angular_frame, text="Bode angular")
        self.notebook.add(self.timeseries_frame, text="Series temporales")

    def set_summary_text(self, text):
        self.summary_text.configure(state=tk.NORMAL)
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert(tk.END, text)
        self.summary_text.configure(state=tk.DISABLED)

    def refresh_runs(self, load_latest=False):
        rid = int(self.robot_var.get())
        runs = available_runs(rid)
        self.run_box["values"] = [p.name for p in runs]
        if runs:
            self.run_var.set(runs[0].name)
            if load_latest:
                self.load_run(runs[0])
        else:
            self.run_var.set("")
            if load_latest:
                self.set_summary_text(f"No hay corridas para robot_{rid}.")

    def load_selected_run(self):
        rid = int(self.robot_var.get())
        name = self.run_var.get()
        if not name:
            return
        self.load_run(RESULTS_DIR / f"robot_{rid}" / name)

    def choose_folder(self):
        selected = filedialog.askdirectory(initialdir=str(RESULTS_DIR))
        if selected:
            self.load_run(Path(selected))

    def clear_plot_frame(self, frame):
        for child in frame.winfo_children():
            child.destroy()

    def load_run(self, run_dir):
        run_dir = Path(run_dir).resolve()
        if not run_dir.exists():
            messagebox.showerror("Carpeta no encontrada", str(run_dir))
            return
        self.current_run = run_dir
        self.bundle = load_result_bundle(run_dir)
        self.path_label.configure(text=str(run_dir))
        self.set_summary_text(build_summary_text(self.bundle))
        self.draw_all_plots()

    def draw_all_plots(self):
        self.draw_bode("lineal", self.bode_linear_frame)
        self.draw_bode("angular", self.bode_angular_frame)
        self.draw_timeseries()

    def draw_bode(self, mode, frame):
        self.clear_plot_frame(frame)
        if self.bundle is None:
            return
        resp = self.bundle["frequency_response"].get(mode, [])
        model = self.bundle["model"].get(mode)
        if not resp:
            ttk.Label(frame, text=f"No hay respuesta en frecuencia para {mode}.").pack(padx=16, pady=16)
            return

        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except ModuleNotFoundError:
            ttk.Label(frame, text="matplotlib no esta instalado. Usa 'Abrir carpeta' para ver PNG.").pack(padx=16, pady=16)
            return

        freqs = np.array([r["freq_hz"] for r in resp], dtype=float)
        gains = np.array([complex(r["gain_real"], r["gain_imag"]) for r in resp], dtype=complex)

        fig = Figure(figsize=(8, 6), dpi=100)
        ax_mag = fig.add_subplot(211)
        ax_phase = fig.add_subplot(212, sharex=ax_mag)
        ax_mag.semilogx(freqs, 20.0 * np.log10(np.abs(gains) + 1e-12), "o", label="medido")
        ax_phase.semilogx(freqs, np.degrees(np.unwrap(np.angle(gains))), "o", label="medido")

        if model:
            dense = np.geomspace(max(min(freqs), 1e-4), max(freqs), 300)
            fit = first_order_delay_response(dense, model["gain"], model["tau_s"], model["delay_s"])
            ax_mag.semilogx(dense, 20.0 * np.log10(np.abs(fit) + 1e-12), "-", label="modelo")
            ax_phase.semilogx(dense, np.degrees(np.unwrap(np.angle(fit))), "-", label="modelo")

        title = "Gv(s): avance" if mode == "lineal" else "Gw(s): giro"
        ax_mag.set_title(title)
        ax_mag.set_ylabel("Magnitud (dB)")
        ax_phase.set_ylabel("Fase (deg)")
        ax_phase.set_xlabel("Frecuencia (Hz)")
        ax_mag.grid(True, which="both")
        ax_phase.grid(True, which="both")
        ax_mag.legend()
        ax_phase.legend()
        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.figures.append(fig)

    def draw_timeseries(self):
        frame = self.timeseries_frame
        self.clear_plot_frame(frame)
        if self.bundle is None or not self.bundle["samples"]:
            ttk.Label(frame, text="No hay samples.csv para graficar.").pack(padx=16, pady=16)
            return

        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except ModuleNotFoundError:
            ttk.Label(frame, text="matplotlib no esta instalado. Usa 'Abrir carpeta' para ver PNG.").pack(padx=16, pady=16)
            return

        samples = self.bundle["samples"]
        t = np.array([s["t"] for s in samples], dtype=float)
        t = t - t[0]
        left = np.array([s["left_cmd"] for s in samples], dtype=float)
        right = np.array([s["right_cmd"] for s in samples], dtype=float)
        v = np.array([s["v_m_s"] for s in samples], dtype=float)
        w = np.array([s["w_rad_s"] for s in samples], dtype=float)

        fig = Figure(figsize=(9, 6), dpi=100)
        ax_cmd = fig.add_subplot(311)
        ax_v = fig.add_subplot(312, sharex=ax_cmd)
        ax_w = fig.add_subplot(313, sharex=ax_cmd)
        ax_cmd.plot(t, left, label="left")
        ax_cmd.plot(t, right, label="right")
        ax_v.plot(t, v, label="v medido", color="#0074D9")
        ax_w.plot(t, w, label="w medido", color="#B10DC9")
        ax_cmd.set_ylabel("Comando (%)")
        ax_v.set_ylabel("v (m/s)")
        ax_w.set_ylabel("w (rad/s)")
        ax_w.set_xlabel("Tiempo (s)")
        for ax in [ax_cmd, ax_v, ax_w]:
            ax.grid(True)
            ax.legend()
        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.figures.append(fig)

    def regenerate_png(self):
        if self.bundle is None:
            return
        msg = plot_results(
            self.bundle["run_dir"],
            self.bundle["frequency_response"],
            self.bundle["model"],
            self.bundle["samples"],
        )
        if msg:
            messagebox.showwarning("Regenerar PNG", msg)
        else:
            messagebox.showinfo("Regenerar PNG", "PNG regenerados correctamente.")

    def open_folder(self):
        if self.current_run:
            os.startfile(str(self.current_run))


def launch_gui(initial_run=None, initial_robot=1):
    root = tk.Tk()
    app = ResultViewerApp(root, initial_run=initial_run, initial_robot=initial_robot)
    root.mainloop()
    return app


def main():
    parser = argparse.ArgumentParser(description="Visualiza resultados de identificacion de modelos.")
    parser.add_argument("--robot", type=int, default=1, help="ID del robot si no se pasa --run.")
    parser.add_argument("--run", type=Path, help="Carpeta de una corrida especifica.")
    parser.add_argument("--regen", action="store_true", help="Regenera los PNG desde JSON/CSV.")
    parser.add_argument("--no-show", action="store_true", help="Solo imprime resumen, no abre interfaz.")
    parser.add_argument("--open-png", action="store_true", help="Abre los PNG con el visor del sistema.")
    args = parser.parse_args()

    run_dir = args.run if args.run else latest_run(args.robot)
    run_dir = run_dir.resolve()
    bundle = load_result_bundle(run_dir)

    if args.regen:
        msg = plot_results(run_dir, bundle["frequency_response"], bundle["model"], bundle["samples"])
        if msg:
            print(msg)

    if args.no_show:
        print_summary(bundle)
        return

    if args.open_png:
        print_summary(bundle)
        show_images_system(run_dir)
        return

    launch_gui(initial_run=run_dir, initial_robot=args.robot)


if __name__ == "__main__":
    main()
