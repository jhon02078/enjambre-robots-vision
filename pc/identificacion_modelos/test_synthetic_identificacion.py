import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analysis import (
    fit_first_order_delay,
    fit_first_order_plus_delay,
    fit_transfer_model,
    first_order_delay_response,
    model_frequency_response,
    pid_recommendations,
    zero_pole_delay_response,
)


def assert_close(value, expected, rel_tol):
    if not math.isclose(value, expected, rel_tol=rel_tol, abs_tol=rel_tol * max(abs(expected), 1e-9)):
        raise AssertionError(f"{value} no esta cerca de {expected}")


def main():
    freqs = np.geomspace(0.05, 2.0, 10)
    true_gain = 0.012
    true_tau = 0.45
    true_delay = 0.08
    h = first_order_delay_response(freqs, true_gain, true_tau, true_delay)
    response = []
    for f, value in zip(freqs, h):
        response.append({
            "freq_hz": float(f),
            "gain_real": float(np.real(value)),
            "gain_imag": float(np.imag(value)),
        })

    model = fit_first_order_delay(response)
    assert_close(model["gain"], true_gain, 0.25)
    assert_close(model["tau_s"], true_tau, 0.35)
    assert_close(model["delay_s"], true_delay, 0.60)

    pid = pid_recommendations(model, model)
    compat = pid["compat_pc_servidor_vision"]
    assert compat["Klin_pct_per_m"] > 0.0
    assert compat["Kang_pct_per_rad"] > 0.0
    assert compat["vmax_sugerido_pct"] == 45.0

    plant = zero_pole_delay_response(
        freqs,
        gain=0.018,
        pole_taus_s=[0.55, 0.14],
        zero_taus_s=[0.08],
        delay_s=0.05,
    )
    response_pz = []
    for f, value in zip(freqs, plant):
        response_pz.append({
            "freq_hz": float(f),
            "gain_real": float(np.real(value)),
            "gain_imag": float(np.imag(value)),
        })

    simple = fit_first_order_plus_delay(response_pz)
    rich = fit_transfer_model(response_pz, max_poles=3, max_zeros=2)
    simple_fit = model_frequency_response(freqs, simple)
    rich_fit = model_frequency_response(freqs, rich)
    simple_err = float(np.mean(np.abs(simple_fit - plant) ** 2))
    rich_err = float(np.mean(np.abs(rich_fit - plant) ** 2))
    if rich_err > simple_err * 0.35:
        raise AssertionError(f"El modelo rico no mejoro suficiente: simple={simple_err}, rico={rich_err}")
    if rich.get("type") != "zero_pole_plus_delay":
        raise AssertionError(f"Se esperaba modelo polo-cero, llego {rich.get('type')}")

    print("OK synthetic identification")


if __name__ == "__main__":
    main()
