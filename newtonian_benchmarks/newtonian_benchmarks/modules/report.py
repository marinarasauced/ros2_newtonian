
#pyright: reportAttributeAccessIssue=false
#pyright: reportArgumentType=false
#pyright: reportGeneralTypeIssues=false

from __future__ import annotations

from typing import Any

import base64
import html
import io
import json
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# columns of the table of trials
SUMMARY = [
    "fell", "rmse_ss_vx", "rmse_ss_vy", "rmse_ss_wz", "height_std", "rp_exceed_frac", "rp_exceed_events", "roll_max", "pitch_max",
    "recovery_time", "mech_power", "cmd_rate_hz", "cmd_gap_max", "cmd_age_p99", "sim_rtf", "sim_overrun_frac",
]

# medians of the scorecard
MEDIANS = ("rmse_ss_vx", "rmse_ss_vy", "rmse_ss_wz", "rp_exceed_frac", "recovery_time", "cmd_age_p99", "sim_rtf")


def write(out: Path) -> Path:
    """
    Writes report.html and summary.json from results.csv.
    """
    out = Path(out)
    df = pd.read_csv(out / "results.csv")

    # scorecard of the run
    s = summary(df=df)
    result: dict[str, Any] = {"all": s}
    parts = [f"<h1>Benchmark {html.escape(out.name)}</h1>", "<h2>Scorecard</h2>", _table(s=s)]

    # one section for each suite
    suites = list(dict.fromkeys(df["suite"].dropna())) if "suite" in df else []
    if len(suites) > 1:
        result["suites"] = {name: summary(df=df[df["suite"] == name]) for name in suites}
        table = pd.DataFrame(result["suites"]).T
        parts.append("<h2>Suites</h2>" + table.to_html(float_format=lambda x: f"{x:.3f}", border=1, na_rep=""))

        for name in suites:
            parts.append(f"<h2>Suite {html.escape(name)}</h2>")
            _section(df=df[df["suite"] == name], parts=parts)
    else:
        _section(df=df, parts=parts)

    (out / "summary.json").write_text(json.dumps(result, indent=1))
    path = out / "report.html"
    path.write_text("<html><meta charset='utf-8'><body style='font-family:sans-serif'>" + "\n".join(parts) + "</body></html>")

    return path


def summary(df: pd.DataFrame) -> dict[str, Any]:
    """
    Returns the scorecard numbers of a set of trials.
    """
    ok = df[df.get("valid", False) == True]  # noqa: E712
    out: dict[str, Any] = {"trials": len(df), "valid": len(ok), "pass_rate": float(df["pass"].mean()) if len(df) else float("nan")}
    if not len(ok):
        return out

    # survival and real-time feasibility
    out["survival_rate"] = float((~ok["fell"].astype(bool)).mean())
    out["rt_ok_rate"] = float(ok["rt_ok"].astype(bool).mean()) if "rt_ok" in ok else float("nan")

    # medians
    for k in MEDIANS:
        if k in ok:
            out[f"{k}_median"] = float(pd.to_numeric(ok[k], errors="coerce").median())

    # recovery, over the trials with pushes
    if "recovered" in ok:
        pushed = ok[ok["pushes"] > 0] if "pushes" in ok else ok.iloc[0:0]
        out["recovery_rate"] = float(pushed["recovered"].astype(bool).mean()) if len(pushed) else float("nan")

    return out


def _png(fig) -> str:
    """
    Returns the figure as a base64 PNG data URI.
    """
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)

    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _table(s: dict[str, Any]) -> str:
    """
    Returns the scorecard as an html table.
    """
    rows = []
    for k, v in s.items():
        value = f"{v:.3f}" if isinstance(v, float) else f"{v}"
        rows.append(f"<tr><td>{k}</td><td>{value}</td></tr>")

    return "<table border=1 cellpadding=4>" + "".join(rows) + "</table>"


def _section(df: pd.DataFrame, parts: list[str]) -> None:
    """
    Adds the plots and the trials table of one suite.
    """
    axes = [c for c in df.columns if c.startswith("axis_") and df[c].notna().any()]
    varied = [a for a in axes if df[a].nunique() > 1]

    # pass rate of each varied axis
    for a in varied:
        parts.append(f"<h3>Pass rate by {a[5:]}</h3><img src='{_bars(df=df, axis=a)}'>")

    # heatmaps of pairs of axes ; the other axes are averaged
    pairs = [(a, b) for i, a in enumerate(varied) for b in varied[i + 1:]]
    for a, b in pairs[:6]:
        others = [c[5:] for c in varied if c not in (a, b)]
        note = f" (mean over {', '.join(others)})" if others else ""

        for metric, title in (("pass", "pass rate"), ("rmse_ss_vx", "steady RMSE vx (m/s)")):
            if metric in df:
                parts.append(f"<h3>{title}: {a[5:]} x {b[5:]}{html.escape(note)}</h3><img src='{_heatmap(df=df, a=a, b=b, metric=metric)}'>")

    # trials
    cols = [c for c in (*axes, "id", "status", "pass", *SUMMARY) if c in df.columns]
    parts.append("<h3>Trials</h3>" + df[cols].to_html(index=False, float_format=lambda x: f"{x:.3f}", border=1, na_rep=""))


def _bars(df: pd.DataFrame, axis: str) -> str:
    """
    Returns the pass rate for each value of 'axis' as a bar plot.
    """
    g = df.groupby(axis)["pass"].mean()

    fig, ax = plt.subplots(figsize=(4.5, 2.6))
    ax.bar([str(i) for i in g.index], g.values, color="#2a7ab0")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("pass rate")
    ax.set_xlabel(axis.replace("axis_", ""))
    plt.xticks(rotation=30, ha="right")

    return _png(fig=fig)


def _heatmap(df: pd.DataFrame, a: str, b: str, metric: str) -> str:
    """
    Returns the mean of 'metric' for each pair of values of 'a' and 'b' as a heatmap.
    """
    pv = df.pivot_table(index=b, columns=a, values=metric, aggfunc="mean")

    fig, ax = plt.subplots(figsize=(0.9 * max(len(pv.columns), 3) + 1.5, 0.6 * max(len(pv.index), 3) + 1.2))
    im = ax.imshow(pv.values.astype(float), cmap="viridis" if metric == "pass" else "magma_r", aspect="auto", origin="lower")

    # labels
    ax.set_xticks(range(len(pv.columns)), [str(c) for c in pv.columns], rotation=30, ha="right")
    ax.set_yticks(range(len(pv.index)), [str(c) for c in pv.index])
    ax.set_xlabel(a[5:])
    ax.set_ylabel(b[5:])

    # value in each cell
    for i in range(pv.shape[0]):
        for j in range(pv.shape[1]):
            v = pv.values[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", color="w", fontsize=8)

    fig.colorbar(im)

    return _png(fig=fig)
