
from __future__ import annotations

from typing import Any

import numpy as np

from newtonian_benchmarks.modules.plugins.metrics.base import METRICS, Context, Metric


@METRICS.register("pushes")
class MetricPushes(Metric):
    """
    Peak deviation and recovery after each push.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the push values to 'out'.
        """
        peak_tilt, peak_dv, rec_times = [], [], []

        for push in ctx.pushes:
            ts = ctx.settle + push["at"]
            te = ts + push["duration"]

            # the run after the push
            window = (ctx.t >= ts) & (ctx.t <= te + ctx.th["recover_window"]) & (ctx.t <= ctx.t_end)
            if not window.any():
                continue

            # peaks
            err = np.hypot(ctx.v_h[:, 0] - ctx.cmd[:, 0], ctx.v_h[:, 1] - ctx.cmd[:, 1])
            peak_tilt.append(float(ctx.tilt[window].max()))
            peak_dv.append(float(err[window].max()))

            rec_times.append(self._recovery_time(ctx=ctx, err=err, ts=ts, te=te))

        out["pushes"] = len(ctx.pushes)
        out["peak_tilt_push"] = float(np.max(peak_tilt)) if peak_tilt else float("nan")
        out["peak_vel_dev_push"] = float(np.max(peak_dv)) if peak_dv else float("nan")

        # recovered : no fall, and every push has a recovery time
        all_found = bool(rec_times) and all(np.isfinite(x) for x in rec_times)
        out["recovered"] = (all_found and not ctx.fell) if ctx.pushes else True
        out["recovery_time"] = float(np.max(rec_times)) if all_found else float("nan")

    def _recovery_time(self, ctx: Context, err: np.ndarray, ts: float, te: float) -> float:
        """
        Returns the time to recover after the push that ends at 'te'.
        """
        th, t = ctx.th, ctx.t

        # errors smoothed over 'recover_smooth' s
        dt = float(np.median(np.diff(t)))
        k = max(1, int(round(th["recover_smooth"] / dt)))
        kernel = np.ones(k) / k
        err_s = np.convolve(err, kernel, mode="same")
        wz_s = np.convolve(np.abs(ctx.wz - ctx.cmd[:, 2]), kernel, mode="same")
        tilt_s = np.convolve(ctx.tilt, kernel, mode="same")

        # level before the push
        before = (t >= ctx.settle + th["transient"]) & (t < ts)
        base_err = float(np.median(err_s[before])) if before.sum() > k else 0.0
        base_wz = float(np.median(wz_s[before])) if before.sum() > k else 0.0
        base_tilt = float(np.median(tilt_s[before])) if before.sum() > k else 0.0

        # samples after the push that are within the margins
        ok = (
            (tilt_s < base_tilt + th["recover_tilt"])
            & (err_s < base_err + th["recover_vel"])
            & (wz_s < base_wz + th["recover_wz"])
            & (t >= te)
        )

        # first run of 'recover_hold' s of samples that are all within the margins
        n = max(1, int(round(th["recover_hold"] / dt)))
        run = np.convolve(ok.astype(float), np.ones(n), mode="full")[:len(ok)]
        hits = np.where((run >= n) & (t <= ctx.t_end + 1.0e-9))[0]
        if not len(hits):
            return float("nan")

        return max(float(t[hits[0]] - (n - 1) * dt - te), 0.0)
