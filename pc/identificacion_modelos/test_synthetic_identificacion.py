import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analysis import fit_first_order_delay, first_order_delay_response, pid_recommendations


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
    print("OK synthetic identification")


if __name__ == "__main__":
    main()
