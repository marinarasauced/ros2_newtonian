
from __future__ import annotations

from typing import Any

import numpy as np

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric, count_episodes, root_mean_square


@METRICS.register("attitude")
class MetricAttitude(Metric):
    """
    Roll and pitch of the base.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the attitude values to 'out'.
        """
        a = ctx.active
        out["roll_rms"] = root_mean_square(x=ctx.roll[a])
        out["pitch_rms"] = root_mean_square(x=ctx.pitch[a])
        out["roll_max"] = float(np.max(np.abs(ctx.roll[a]))) if a.any() else float("nan")
        out["pitch_max"] = float(np.max(np.abs(ctx.pitch[a]))) if a.any() else float("nan")
        exceed = a & ((np.abs(ctx.roll) > ctx.th["rp_limit"]) | (np.abs(ctx.pitch) > ctx.th["rp_limit"]))
        out["rp_exceed_frac"] = float(exceed[a].mean()) if a.any() else float("nan")
        out["rp_exceed_events"] = count_episodes(mask=exceed[a]) if a.any() else 0
