import json
import math
from pathlib import Path

import numpy as np


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def estimate_single_frequency(t, u, y, freq_hz):
    t = np.asarray(t, dtype=float)
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    valid = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t, u, y = t[valid], u[valid], y[valid]
    if len(t) < 12:
        raise ValueError("No hay suficientes muestras para estimar la frecuencia.")

    u = u - np.mean(u)
    y = y - np.mean(y)
    if np.std(u) < 1e-9:
        raise ValueError("La entrada no tiene variacion suficiente.")

    w = 2.0 * np.pi * float(freq_hz)
    basis = np.exp(-1j * w * (t - t[0]))
    u_complex = (2.0 / len(t)) * np.sum(u * basis)
    y_complex = (2.0 / len(t)) * np.sum(y * basis)
    gain = y_complex / (u_complex + 1e-12)
    return {
        "freq_hz": float(freq_hz),
        "omega_rad_s": float(w),
        "gain_real": float(np.real(gain)),
        "gain_imag": float(np.imag(gain)),
        "magnitude": float(abs(gain)),
        "phase_rad": float(np.angle(gain)),
        "phase_deg": float(np.degrees(np.angle(gain))),
        "n_samples": int(len(t)),
    }


def estimate_frequency_response(samples, mode, settle_cycles):
    groups = {}
    for row in samples:
        if row.get("mode") != mode:
            continue
        freq = float(row["freq_hz"])
        groups.setdefault(freq, []).append(row)

    response = []
    for freq, rows in sorted(groups.items()):
        settle_s = float(settle_cycles) / freq
        usable = [r for r in rows if float(r["segment_t"]) >= settle_s]
        if len(usable) < 12:
            usable = rows

        t = [float(r["segment_t"]) for r in usable]
        if mode == "lineal":
            u = [(float(r["left_cmd"]) + float(r["right_cmd"])) * 0.5 for r in usable]
            y = [float(r["v_m_s"]) for r in usable]
        else:
            u = [(float(r["right_cmd"]) - float(r["left_cmd"])) * 0.5 for r in usable]
            y = [float(r["w_rad_s"]) for r in usable]

        try:
            response.append(estimate_single_frequency(t, u, y, freq))
        except ValueError:
            continue
    return response


def first_order_delay_response(freq_hz, gain, tau_s, delay_s):
    freq_hz = np.asarray(freq_hz, dtype=float)
    w = 2.0 * np.pi * freq_hz
    return gain * np.exp(-1j * w * delay_s) / (1.0 + 1j * w * tau_s)


def zero_pole_delay_response(freq_hz, gain, pole_taus_s, zero_taus_s=None, delay_s=0.0):
    freq_hz = np.asarray(freq_hz, dtype=float)
    pole_taus_s = np.asarray([] if pole_taus_s is None else pole_taus_s, dtype=float)
    zero_taus_s = np.asarray([] if zero_taus_s is None else zero_taus_s, dtype=float)
    s = 1j * 2.0 * np.pi * freq_hz

    response = np.full_like(s, complex(gain), dtype=complex)
    for tau_z in zero_taus_s:
        response *= 1.0 + s * tau_z
    for tau_p in pole_taus_s:
        response /= 1.0 + s * max(float(tau_p), 1e-9)
    response *= np.exp(-s * float(delay_s))
    return response


def model_frequency_response(freq_hz, model):
    if not model:
        return np.zeros_like(np.asarray(freq_hz, dtype=float), dtype=complex)

    if model.get("type") == "zero_pole_plus_delay":
        return zero_pole_delay_response(
            freq_hz,
            model.get("gain", 0.0),
            model.get("pole_time_constants_s", []),
            model.get("zero_time_constants_s", []),
            model.get("delay_s", 0.0),
        )

    return first_order_delay_response(
        freq_hz,
        model.get("gain", 0.0),
        model.get("tau_s", 0.0),
        model.get("delay_s", 0.0),
    )


def _unwrap_phase(phases):
    return np.unwrap(np.asarray(phases, dtype=float))


def _fit_with_scipy(freq_hz, measured):
    from scipy.optimize import least_squares

    mag0 = max(float(np.median(np.abs(measured[: max(1, min(3, len(measured)))]))), 1e-6)
    x0 = np.array([mag0, 0.35, 0.03], dtype=float)

    measured_phase = _unwrap_phase(np.angle(measured))

    def residual(x):
        gain, tau_s, delay_s = x
        pred = first_order_delay_response(freq_hz, gain, tau_s, delay_s)
        mag_res = np.log(np.abs(pred) + 1e-12) - np.log(np.abs(measured) + 1e-12)
        phase_res = _unwrap_phase(np.angle(pred)) - measured_phase
        return np.r_[mag_res, 0.45 * phase_res]

    result = least_squares(
        residual,
        x0,
        bounds=([1e-8, 0.005, 0.0], [100.0, 20.0, 3.0]),
        max_nfev=2000,
    )
    return result.x, float(np.mean(residual(result.x) ** 2)), "scipy"


def _fit_with_numpy_grid(freq_hz, measured):
    taus = np.geomspace(0.02, 8.0, 80)
    delays = np.linspace(0.0, 1.2, 80)
    best = None

    for tau_s in taus:
        base = first_order_delay_response(freq_hz, 1.0, tau_s, 0.0)
        for delay_s in delays:
            h = base * np.exp(-1j * 2.0 * np.pi * freq_hz * delay_s)
            gain = max(float(np.real(np.vdot(h, measured) / (np.vdot(h, h) + 1e-12))), 1e-8)
            pred = gain * h
            mag_res = np.log(np.abs(pred) + 1e-12) - np.log(np.abs(measured) + 1e-12)
            phase_res = _unwrap_phase(np.angle(pred)) - _unwrap_phase(np.angle(measured))
            err = float(np.mean(np.r_[mag_res, 0.45 * phase_res] ** 2))
            if best is None or err < best[0]:
                best = (err, gain, tau_s, delay_s)

    err, gain, tau_s, delay_s = best
    return np.array([gain, tau_s, delay_s], dtype=float), err, "numpy_grid"


def fit_first_order_plus_delay(response):
    if len(response) < 2:
        raise ValueError("Se necesitan al menos dos frecuencias para ajustar el modelo.")

    freq_hz = np.array([r["freq_hz"] for r in response], dtype=float)
    measured = np.array([complex(r["gain_real"], r["gain_imag"]) for r in response], dtype=complex)
    order = np.argsort(freq_hz)
    freq_hz = freq_hz[order]
    measured = measured[order]

    try:
        params, mse, method = _fit_with_scipy(freq_hz, measured)
    except Exception:
        params, mse, method = _fit_with_numpy_grid(freq_hz, measured)

    gain, tau_s, delay_s = [float(v) for v in params]
    return {
        "type": "first_order_plus_delay",
        "gain": gain,
        "tau_s": tau_s,
        "dominant_tau_s": tau_s,
        "delay_s": delay_s,
        "fit_mse": float(mse),
        "fit_method": method,
    }


def _normalize_response(response):
    if len(response) < 2:
        raise ValueError("Se necesitan al menos dos frecuencias para ajustar el modelo.")

    freq_hz = np.array([r["freq_hz"] for r in response], dtype=float)
    measured = np.array([complex(r["gain_real"], r["gain_imag"]) for r in response], dtype=complex)
    valid = np.isfinite(freq_hz) & np.isfinite(measured.real) & np.isfinite(measured.imag) & (freq_hz > 0.0)
    freq_hz = freq_hz[valid]
    measured = measured[valid]
    if len(freq_hz) < 2:
        raise ValueError("No hay suficientes puntos validos para ajustar el modelo.")

    order = np.argsort(freq_hz)
    return freq_hz[order], measured[order]


def _model_mse(freq_hz, measured, model):
    pred = model_frequency_response(freq_hz, model)
    mag_res = np.log(np.abs(pred) + 1e-12) - np.log(np.abs(measured) + 1e-12)
    phase_res = _unwrap_phase(np.angle(pred)) - _unwrap_phase(np.angle(measured))
    return float(np.mean(np.r_[mag_res, 0.45 * phase_res] ** 2))


def _pole_zero_metadata(gain, pole_taus, zero_taus, delay_s, fit_mse, fit_method, candidate_table):
    pole_taus = [float(max(t, 1e-9)) for t in pole_taus]
    zero_taus = [float(t) for t in zero_taus]
    dominant_tau = max(pole_taus) if pole_taus else 0.0
    poles_rad_s = [-1.0 / t for t in pole_taus if abs(t) > 1e-12]
    zeros_rad_s = [-1.0 / t for t in zero_taus if abs(t) > 1e-12]
    numerator_terms = [f"(1 + {t:.6g}s)" for t in zero_taus]
    denominator_terms = [f"(1 + {t:.6g}s)" for t in pole_taus]

    return {
        "type": "zero_pole_plus_delay",
        "gain": float(gain),
        "tau_s": float(dominant_tau),
        "dominant_tau_s": float(dominant_tau),
        "delay_s": float(delay_s),
        "n_poles": len(pole_taus),
        "n_zeros": len(zero_taus),
        "pole_time_constants_s": pole_taus,
        "zero_time_constants_s": zero_taus,
        "poles_rad_s": [float(v) for v in poles_rad_s],
        "zeros_rad_s": [float(v) for v in zeros_rad_s],
        "equation": (
            f"{gain:.8g} * "
            f"{' * '.join(numerator_terms) if numerator_terms else '1'} / "
            f"{' * '.join(denominator_terms) if denominator_terms else '1'} "
            f"* exp(-{delay_s:.6g}s)"
        ),
        "fit_mse": float(fit_mse),
        "fit_method": fit_method,
        "candidate_models": candidate_table,
    }


def _fit_zero_pole_with_scipy(freq_hz, measured, n_poles, n_zeros, base_model):
    from scipy.optimize import least_squares

    measured_phase = _unwrap_phase(np.angle(measured))
    mag0 = max(float(np.median(np.abs(measured[: max(1, min(3, len(measured)))]))), 1e-8)
    base_gain = abs(float(base_model.get("gain", mag0))) if base_model else mag0
    base_tau = max(float(base_model.get("tau_s", 0.35)) if base_model else 0.35, 0.02)
    base_delay = max(float(base_model.get("delay_s", 0.03)) if base_model else 0.03, 0.0)

    sign_candidates = [-1.0] if measured[0].real < 0.0 else [1.0]

    best = None
    lower = np.array(
        [math.log(1e-8)]
        + [math.log(0.005)] * n_poles
        + [-8.0] * n_zeros
        + [0.0],
        dtype=float,
    )
    upper = np.array(
        [math.log(200.0)]
        + [math.log(30.0)] * n_poles
        + [8.0] * n_zeros
        + [3.0],
        dtype=float,
    )

    def unpack(x, sign):
        gain = sign * math.exp(float(x[0]))
        pole_taus = np.exp(x[1:1 + n_poles])
        zero_taus = np.asarray(x[1 + n_poles:1 + n_poles + n_zeros], dtype=float)
        delay_s = float(x[-1])
        return gain, pole_taus, zero_taus, delay_s

    def residual(x, sign):
        gain, pole_taus, zero_taus, delay_s = unpack(x, sign)
        pred = zero_pole_delay_response(freq_hz, gain, pole_taus, zero_taus, delay_s)
        mag_res = np.log(np.abs(pred) + 1e-12) - np.log(np.abs(measured) + 1e-12)
        phase_res = _unwrap_phase(np.angle(pred)) - measured_phase

        # Regularizacion leve: evita cancelaciones polo-cero enormes cuando hay pocos puntos.
        reg_poles = 0.015 * (np.log(pole_taus / base_tau))
        reg_zeros = 0.01 * (zero_taus / max(base_tau, 1e-3))
        return np.r_[mag_res, 0.45 * phase_res, reg_poles, reg_zeros]

    pole_guesses = [
        np.full(n_poles, base_tau, dtype=float),
        np.geomspace(max(base_tau * 0.2, 0.01), max(base_tau * 3.0, 0.03), n_poles),
    ]
    zero_guesses = [
        np.zeros(n_zeros, dtype=float),
        np.full(n_zeros, min(base_tau * 0.25, 1.0), dtype=float),
    ]

    for sign in sign_candidates:
        for poles0 in pole_guesses:
            for zeros0 in zero_guesses:
                x0 = np.array(
                    [math.log(max(base_gain, 1e-8))]
                    + [math.log(max(float(v), 0.005)) for v in poles0]
                    + [float(v) for v in zeros0]
                    + [base_delay],
                    dtype=float,
                )
                x0 = np.clip(x0, lower + 1e-9, upper - 1e-9)
                try:
                    result = least_squares(
                        lambda x: residual(x, sign),
                        x0,
                        bounds=(lower, upper),
                        max_nfev=1400,
                        xtol=1e-8,
                        ftol=1e-8,
                    )
                except Exception:
                    continue

                gain, pole_taus, zero_taus, delay_s = unpack(result.x, sign)
                candidate = {
                    "type": "zero_pole_plus_delay",
                    "gain": gain,
                    "pole_time_constants_s": [float(v) for v in pole_taus],
                    "zero_time_constants_s": [float(v) for v in zero_taus],
                    "delay_s": delay_s,
                }
                mse = _model_mse(freq_hz, measured, candidate)
                if best is None or mse < best[0]:
                    best = (mse, gain, pole_taus, zero_taus, delay_s)

    if best is None:
        raise RuntimeError("No se pudo ajustar modelo polo-cero con scipy.")
    return best


def fit_transfer_model(response, max_poles=3, max_zeros=2):
    freq_hz, measured = _normalize_response(response)
    first_order = fit_first_order_plus_delay(response)
    first_order["candidate_models"] = [{
        "type": first_order["type"],
        "n_poles": 1,
        "n_zeros": 0,
        "fit_mse": first_order["fit_mse"],
        "fit_method": first_order["fit_method"],
    }]

    try:
        import scipy  # noqa: F401
    except Exception:
        first_order["fit_method"] = f"{first_order['fit_method']} (sin scipy para polos/ceros)"
        return first_order

    n_points = len(freq_hz)
    max_poles = int(max(1, min(max_poles, 4, max(1, n_points - 1))))
    max_zeros = int(max(0, min(max_zeros, 3)))

    candidates = [{
        "type": first_order["type"],
        "n_poles": 1,
        "n_zeros": 0,
        "fit_mse": float(first_order["fit_mse"]),
        "selection": "referencia",
    }]
    best_model = first_order
    best_mse = float(first_order["fit_mse"])

    for n_poles in range(1, max_poles + 1):
        for n_zeros in range(0, min(max_zeros, max(0, n_poles - 1)) + 1):
            if n_poles == 1 and n_zeros == 0:
                continue
            n_params = 1 + n_poles + n_zeros + 1
            if 2 * n_points <= n_params:
                continue
            try:
                mse, gain, pole_taus, zero_taus, delay_s = _fit_zero_pole_with_scipy(
                    freq_hz, measured, n_poles, n_zeros, first_order
                )
            except Exception:
                continue
            candidates.append({
                "type": "zero_pole_plus_delay",
                "n_poles": int(n_poles),
                "n_zeros": int(n_zeros),
                "fit_mse": float(mse),
                "gain": float(gain),
                "delay_s": float(delay_s),
            })
            if mse < best_mse:
                best_mse = mse
                best_model = _pole_zero_metadata(
                    gain,
                    pole_taus,
                    zero_taus,
                    delay_s,
                    mse,
                    f"scipy_least_squares_pz_p{n_poles}_z{n_zeros}",
                    [],
                )

    candidates = sorted(candidates, key=lambda c: c.get("fit_mse", float("inf")))
    if best_model.get("type") == "zero_pole_plus_delay":
        best_model["candidate_models"] = candidates
    else:
        best_model["candidate_models"] = candidates
    best_model["first_order_reference"] = {
        key: first_order[key]
        for key in ["type", "gain", "tau_s", "delay_s", "fit_mse", "fit_method"]
        if key in first_order
    }
    return best_model


def fit_first_order_delay(response):
    return fit_transfer_model(response)


def pid_recommendations(linear_model=None, angular_model=None):
    result = {
        "lineal": None,
        "angular": None,
        "compat_pc_servidor_vision": {},
    }

    if linear_model:
        k = max(float(linear_model["gain"]), 1e-6)
        tau = max(float(linear_model["tau_s"]), 0.05)
        delay = max(float(linear_model["delay_s"]), 0.0)
        desired_tau = max(0.8, 1.5 * (tau + delay))
        kp = 1.0 / (k * desired_tau)
        ki = kp / max(4.0 * desired_tau, 1e-6)
        kd = 0.0
        result["lineal"] = {
            "kp_pct_per_m": float(np.clip(kp, 5.0, 250.0)),
            "ki_pct_per_m_s": float(np.clip(ki, 0.0, 80.0)),
            "kd_pct_s_per_m": kd,
            "desired_tau_s": desired_tau,
        }
        result["compat_pc_servidor_vision"]["Klin_pct_per_m"] = result["lineal"]["kp_pct_per_m"]

    if angular_model:
        k = max(float(angular_model["gain"]), 1e-6)
        tau = max(float(angular_model["tau_s"]), 0.03)
        delay = max(float(angular_model["delay_s"]), 0.0)
        desired_tau = max(0.35, 1.3 * (tau + delay))
        kp = 1.0 / (k * desired_tau)
        ki = kp / max(5.0 * desired_tau, 1e-6)
        kd = 0.25 * kp * tau
        result["angular"] = {
            "kp_pct_per_rad": float(np.clip(kp, 2.0, 120.0)),
            "ki_pct_per_rad_s": float(np.clip(ki, 0.0, 80.0)),
            "kd_pct_s_per_rad": float(np.clip(kd, 0.0, 40.0)),
            "desired_tau_s": desired_tau,
        }
        result["compat_pc_servidor_vision"]["Kang_pct_per_rad"] = result["angular"]["kp_pct_per_rad"]
        result["compat_pc_servidor_vision"]["Kd_ang_pct"] = result["angular"]["kd_pct_s_per_rad"]

    if linear_model or angular_model:
        result["compat_pc_servidor_vision"]["vmax_sugerido_pct"] = 45.0

    return result


def combined_differential_model(linear_model=None, angular_model=None):
    return {
        "type": "differential_drive_decoupled_velocity_model",
        "inputs": {
            "u_v_pct": "(left_cmd + right_cmd) / 2",
            "u_w_pct": "(right_cmd - left_cmd) / 2",
        },
        "outputs": {
            "v_m_s": "linear velocity projected on robot heading",
            "w_rad_s": "unwrapped yaw rate",
        },
        "transfer_matrix": {
            "v_m_s/u_v_pct": linear_model,
            "v_m_s/u_w_pct": None,
            "w_rad_s/u_v_pct": None,
            "w_rad_s/u_w_pct": angular_model,
        },
        "notes": [
            "Es un solo modelo del robot escrito en coordenadas desacopladas avance/giro.",
            "La matriz se asume diagonal para control diferencial basico; acoplamientos cruzados se ignoran en esta version.",
        ],
    }


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def plot_results(out_dir, frequency_response, models, samples):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib no disponible: {exc}"

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for mode, filename in [("lineal", "bode_lineal.png"), ("angular", "bode_angular.png")]:
        resp = frequency_response.get(mode, [])
        if not resp:
            continue
        freqs = np.array([r["freq_hz"] for r in resp], dtype=float)
        gains = np.array([complex(r["gain_real"], r["gain_imag"]) for r in resp], dtype=complex)
        model = models.get(mode)

        fig, axes = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
        axes[0].semilogx(freqs, 20.0 * np.log10(np.abs(gains) + 1e-12), "o", label="medido")
        axes[1].semilogx(freqs, np.degrees(_unwrap_phase(np.angle(gains))), "o", label="medido")
        if model:
            dense = np.geomspace(max(min(freqs), 1e-4), max(freqs), 200)
            fit = model_frequency_response(dense, model)
            axes[0].semilogx(dense, 20.0 * np.log10(np.abs(fit) + 1e-12), "-", label="ajuste")
            axes[1].semilogx(dense, np.degrees(_unwrap_phase(np.angle(fit))), "-", label="ajuste")
        axes[0].set_ylabel("Magnitud (dB)")
        axes[1].set_ylabel("Fase (deg)")
        axes[1].set_xlabel("Frecuencia (Hz)")
        axes[0].grid(True, which="both")
        axes[1].grid(True, which="both")
        axes[0].legend()
        fig.tight_layout()
        fig.savefig(out_dir / filename, dpi=150)
        plt.close(fig)

    if samples:
        t = np.array([s["t"] for s in samples], dtype=float)
        t = t - t[0]
        fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        axes[0].plot(t, [s["left_cmd"] for s in samples], label="L")
        axes[0].plot(t, [s["right_cmd"] for s in samples], label="R")
        axes[1].plot(t, [s["v_m_s"] for s in samples], label="v")
        axes[2].plot(t, [s["w_rad_s"] for s in samples], label="w")
        axes[0].set_ylabel("Comando (%)")
        axes[1].set_ylabel("v (m/s)")
        axes[2].set_ylabel("w (rad/s)")
        axes[2].set_xlabel("Tiempo (s)")
        for ax in axes:
            ax.grid(True)
            ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "timeseries.png", dpi=150)
        plt.close(fig)
    return None
