
from __future__ import annotations

from typing import Any

import numpy as np

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric, root_mean_square


@METRICS.register("tracking")
class MetricTracking(Metric):
    """
    Tracking error of the command.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the tracking values to 'out'.
        """
        for name, est, ref in (("vx", ctx.v_h[:, 0], ctx.cmd[:, 0]), ("vy", ctx.v_h[:, 1], ctx.cmd[:, 1]), ("wz", ctx.wz, ctx.cmd[:, 2])):
            out[f"rmse_{name}"] = root_mean_square(x=(est - ref)[ctx.active])
            out[f"rmse_ss_{name}"] = root_mean_square(x=(est - ref)[ctx.steady])
        height = ctx.pos[:, 2]
        out["height_mean"] = float(np.mean(height[ctx.active])) if ctx.active.any() else float("nan")
        out["height_std"] = float(np.std(height[ctx.active])) if ctx.active.any() else float("nan")
        out["rmse_vel"] = float(np.sqrt(out["rmse_vx"] ** 2 + out["rmse_vy"] ** 2)) if ctx.active.any() else float("nan")
        out["distance"] = float(np.linalg.norm(ctx.pos[ctx.active][-1, :2] - ctx.pos[ctx.active][0, :2])) if ctx.active.sum() > 1 else 0.0
