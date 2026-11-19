
from __future__ import annotations

from typing import Any

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric


@METRICS.register("realtime")
class MetricRealtime(Metric):
    """
    Real-time check of the simulator and the controller.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the real-time values to 'out'.
        """
        timing, th = ctx.timing, ctx.th
        for k in ("rtf", "step_ms_p50", "step_ms_p99", "step_ms_max", "overruns", "frames"):
            if k in timing:
                out[f"sim_{k}"] = timing[k]
        frames = max(float(timing.get("frames", 0)), 1.0)
        out["sim_overrun_frac"] = float(timing.get("overruns", 0)) / frames
        rtf_ok = float(timing.get("rtf", 0.0)) >= 0.97 * ctx.rtf_target
        ctl_ok = out.get("cmd_gap_max", float("inf")) < th["gap_limit"] and out.get("cmd_missed_frac", 1.0) < 0.01 and out.get("cmd_rate_ratio", 0.0) >= 0.9
        out["rt_ok"] = bool(rtf_ok and ctl_ok and out["sim_overrun_frac"] < th["overrun_frac"])
