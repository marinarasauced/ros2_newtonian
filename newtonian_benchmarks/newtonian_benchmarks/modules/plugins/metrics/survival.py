
from __future__ import annotations

from typing import Any

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric


@METRICS.register("survival")
class MetricSurvival(Metric):
    """
    Fall, time to fall, and length of the run.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the survival values to 'out'.
        """
        out.update({"fell": ctx.fell, "time_to_fall": ctx.t_fall - ctx.settle if ctx.fell else float("nan"), "duration_run": ctx.t_end - ctx.settle})
