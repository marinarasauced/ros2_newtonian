
from __future__ import annotations

from typing import Any


def find_pushes(params: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Returns the pushes of 'params' that have a force.
    """
    p = params.get("push")
    if not p or isinstance(p, str):
        return []

    p = [p] if isinstance(p, dict) else p

    return [x for x in p if x.get("force", 0)]


def to_request(params: dict[str, Any], sensors: list[str]) -> dict[str, Any]:
    """
    Returns the 'configure' request of 'params'.
    """
    settle = float(params.get("settle", 1.0))
    req: dict[str, Any] = {"reset": True, "defaults": True}

    # physics
    if "friction" in params:
        req["friction"] = float(params["friction"])
    if "payload" in params:
        req["payload_mass"] = float(params["payload"])

    # start state
    if "start" in params:
        req["state"] = params["start"]

    # pushes: a direction '+x' or 'rz' with its force or torque, from the end of the settle time
    pushes = find_pushes(params=params)
    if pushes:
        req["push"] = []
        for p in pushes:
            direction = str(p["dir"])
            sign = -1.0 if direction.startswith("-") else 1.0
            axis = direction.lstrip("+-")

            key = "force"
            if axis.startswith("r"):
                key, axis = "torque", axis[1:]

            vec = [0.0, 0.0, 0.0]
            vec["xyz".index(axis)] = sign * float(p["force"])
            req["push"].append({key: vec, "start": settle + float(p.get("at", 3.0)), "duration": float(p.get("duration", 0.2)), "body": p.get("body")})

    # latency of the commands and of the sensors; jitter and drop apply to both
    lat = {k: float(params[k]) for k in ("jitter", "drop") if k in params}
    if "cmd_latency" in params or lat:
        req["inputs"] = {"*": {"latency": float(params.get("cmd_latency", 0.0)), **lat}}
    if "sensor_latency" in params or lat:
        req["sensors"] = {s: {"latency": float(params.get("sensor_latency", 0.0)), **lat} for s in sensors}

    # speed of the run
    if "rtf" in params:
        req["rtf"] = float(params["rtf"])

    return req
