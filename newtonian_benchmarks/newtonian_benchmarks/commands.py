
from __future__ import annotations

from typing import Any

import math


def command(t: float, p: dict[str, Any]) -> tuple[float, float, float, float]:
    """
    Returns a locomotion command (vx, vy, yaw_rate, height) of the command profile at the trial time in seconds.
    """
    height = float(p["height"])
    settle = float(p.get("settle", 1.0))
    if t < settle:
        return 0.0, 0.0, 0.0, height

    u = t - settle
    target = (float(p.get("vx", 0.0)), float(p.get("vy", 0.0)), float(p.get("yaw_rate", 0.0)))
    profile = p.get("profile", "step")

    if profile == "step":
        k = 1.0
    elif profile == "ramp":
        k = min(u / max(float(p.get("ramp", 2.0)), 1.0e-6), 1.0)
    elif profile == "sine":
        k = math.sin(2.0 * math.pi * float(p.get("frequency", 0.25)) * u)
    elif profile == "stop_and_go":
        k = 1.0 if u < float(p.get("go", 3.0)) else 0.0
    else:
        raise ValueError(f"unknown profile {profile!r}")

    return target[0] * k, target[1] * k, target[2] * k, height
