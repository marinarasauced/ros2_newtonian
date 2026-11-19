
from typing import Any

import numpy as np

from .base import DEFAULTS, METRICS, Context, Metric
from . import attitude, controller, power, pushes, realtime, survival, tracking

# order of the metrics: a metric may read the values of the ones before it
ORDER = ("survival", "tracking", "attitude", "pushes", "power", "controller", "realtime")


def evaluate(
    odom: np.ndarray,
    js: list,
    cmd_rx: np.ndarray,
    profile: dict[str, Any],
    duration: float,
    pushes: list[dict[str, Any]],
    timing: dict[str, Any],
    rtf_target: float,
    thresholds: dict[str, float] | None = None,
) -> dict[str, Any]:
    """
    Returns the metrics of one trial.
    """
    th = {**DEFAULTS, **(thresholds or {})}
    if len(odom) < 10:
        return {"fell": True, "valid": False, "note": "no odometry"}

    # signals that the metrics share
    ctx = Context.from_trial(odom=odom, js=js, cmd_rx=cmd_rx, profile=profile, pushes=pushes, timing=timing, rtf_target=rtf_target, th=th)

    # each metric adds its values
    out: dict[str, Any] = {}
    for name in ORDER:
        METRICS.get(name)(name=name, p=th).evaluate(ctx=ctx, out=out)
    out["valid"] = True

    return out


def is_passed(m: dict[str, Any], rule: dict[str, float]) -> bool:
    """
    Returns True if 'm' satisfies 'rule'.
    """
    if not m.get("valid", False) or m.get("fell", True):
        return False

    # limits: every 'max_<metric>' key
    for key, limit in rule.items():
        if key.startswith("max_"):
            value = m.get(key[4:])
            if value is None or not np.isfinite(value) or value > limit:
                return False

    # recovery after the pushes
    if rule.get("require_recovery", 0) and not m.get("recovered", False):
        return False

    return True


__all__ = ["DEFAULTS", "METRICS", "Context", "Metric", "ORDER", "evaluate", "is_passed"]
