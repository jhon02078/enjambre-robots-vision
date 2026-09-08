import json
import math
from pathlib import Path


SUPPORTED_ALIGNMENT_METHODS = {"translation_bias_after_parallax"}


def load_localization_alignment(path):
    source = Path(path).expanduser()
    data = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("La alineacion debe ser un objeto JSON")
    method = data.get("method")
    if method not in SUPPORTED_ALIGNMENT_METHODS:
        raise ValueError(f"Metodo de alineacion no soportado: {method}")
    for key in ("offset_x_m", "offset_y_m"):
        value = float(data[key])
        if not math.isfinite(value) or abs(value) > 0.25:
            raise ValueError(f"Offset invalido en {key}")
        data[key] = value
    data["path"] = str(source.resolve())
    return data


def apply_localization_alignment(x_m, y_m, alignment, enabled=True):
    if not enabled or not alignment or not alignment.get("enabled", True):
        return float(x_m), float(y_m)
    return (
        float(x_m) + float(alignment["offset_x_m"]),
        float(y_m) + float(alignment["offset_y_m"]),
    )
