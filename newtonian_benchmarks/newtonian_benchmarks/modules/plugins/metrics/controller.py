
from __future__ import annotations

from typing import Any

import numpy as np

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric


@METRICS.register("controller")
class MetricController(Metric):
    """
    Timing of the commands of the controller.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the timing values to 'out'.
        """
        rx = ctx.cmd_rx
        if len(rx) <= 10:
            out["cmd_gap_max"] = float("inf")
            return
        ta, ts = rx[:, 0], rx[:, 1]
        sel = (ta >= ctx.settle) & (ta <= ctx.t_end)
        if sel.sum() <= 5:
            return
        gaps = np.diff(ta[sel])
        age = (ta - ts)[sel]
        med = float(np.median(gaps)) if len(gaps) else float("nan")
        out["cmd_rate_hz"] = float(1.0 / med) if med and med > 0 else float("nan")
        out["cmd_gap_max"] = float(gaps.max())
        out["cmd_gap_p99"] = float(np.percentile(gaps, 99))
        out["cmd_age_p50"] = float(np.percentile(age, 50))
        out["cmd_age_p99"] = float(np.percentile(age, 99))
        period = float(ctx.th.get("cmd_period") or med)
        out["cmd_missed"] = int(np.sum(gaps > 1.5 * period + 0.002))
        out["cmd_missed_frac"] = out["cmd_missed"] / max(len(gaps), 1)
        out["cmd_rate_ratio"] = out["cmd_rate_hz"] * period
