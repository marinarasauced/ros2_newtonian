
from __future__ import annotations

from typing import Any

import numpy as np

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric


@METRICS.register("power")
class MetricPower(Metric):
    """
    Mechanical power and peak effort of the joints.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the power values to 'out'.
        """
        if not ctx.js:
            return
        tj = np.array([row[0] for row in ctx.js])
        sel = (tj >= ctx.settle) & (tj <= ctx.t_end)
        try:
            power = np.array([np.sum(np.abs(np.asarray(row[1]) * np.asarray(row[2]))) for row in ctx.js])
            out["mech_power"] = float(power[sel].mean()) if sel.any() else float("nan")
            effort = np.array([np.max(np.abs(row[2])) if len(row[2]) else 0.0 for row in ctx.js])
            out["effort_max"] = float(effort[sel].max()) if sel.any() else float("nan")
        except Exception:
            pass
