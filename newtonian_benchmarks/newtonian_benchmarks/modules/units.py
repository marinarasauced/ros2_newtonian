
from __future__ import annotations

from typing import Any


def resolve_terrain(terrain: Any, height: float) -> Any:
    """
    Returns 'terrain' with the keys that end in '_rel' scaled by 'height'.
    """
    if not isinstance(terrain, dict):
        return terrain

    out = {}
    for k, v in terrain.items():
        if k.endswith("_rel"):
            out[k[:-4]] = round(float(v) * height, 5)
        else:
            out[k] = v

    return out


def resolve_parameters(params: dict[str, Any], mass: float, height: float, gravity: float = 9.81) -> dict[str, Any]:
    """
    Returns 'params' with the push forces and the payload in absolute units.
    """
    out = dict(params)

    # payload
    if "payload_frac" in out:
        out["payload"] = float(out.pop("payload_frac")) * mass

    # pushes
    pushes = out.get("push")
    if pushes and not isinstance(pushes, str):
        resolved = []
        for p in [pushes] if isinstance(pushes, dict) else pushes:
            p = dict(p)

            if "force_bw" in p:
                is_rotation = str(p["dir"]).lstrip("+-").startswith("r")
                scale = mass * gravity * (height if is_rotation else 1.0)
                p["force"] = float(p.pop("force_bw")) * scale

            resolved.append(p)
        out["push"] = resolved

    return out
