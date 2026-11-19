#pyright: reportAttributeAccessIssue=false
#pyright: reportCallIssue=false
#pyright: reportArgumentType=false
#pyright: reportGeneralTypeIssues=false

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from newtonian_benchmarks.modules.plugins.metrics import DEFAULTS
from newtonian_benchmarks.modules.plugins.metrics.base import euler


# failed trials with a timeline for each controller
TIMELINES = 3

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf"]


def reason_of(df: pd.DataFrame) -> pd.Series:
    """
    Returns the reason of failure of each trial, and an empty string for the trials that passed.
    """
    if "fail_reason" in df:
        return pd.Series(df["fail_reason"].fillna("").astype(str), index=df.index)

    fell = df["fell"].astype(bool) if "fell" in df else pd.Series(False, index=df.index)

    return pd.Series(np.where(df["pass"].astype(bool), "", np.where(fell, "fell", "other")), index=df.index)


def failure_timelines(out: Path, df: pd.DataFrame, controller: str | None, count: int = TIMELINES) -> list[tuple[str, str]]:
    """
    Writes the timeline of the first failed trials and returns their label and link.
    """
    links = []
    failed = df[reason_of(df=df) != ""]

    for _, row in failed.iterrows():
        if len(links) >= count:
            break

        trace = out / "traces" / (controller or "") / f"{row['id']}.npz"
        if not trace.exists():
            continue

        axes = ", ".join(f"{c[5:]} {row[c]}" for c in df.columns if c.startswith("axis_") and df[c].nunique(dropna=False) > 1)
        label = f"{row['id']} : {axes} : {reason_of(df=failed.loc[[row.name]]).iloc[0]}"
        path = out / "plots" / f"timeline_{controller or 'run'}_{row['id']}.html"
        _timeline(trace=trace, row=row, title=f"{controller or ''} {label}".strip(), path=path)
        links.append((label, f"plots/{path.name}"))

    return links


def pass_rate_by_axis(out: Path, frames: dict[str, pd.DataFrame]) -> str | None:
    """
    Writes the pass rate against each axis, one line for each controller, and returns the link.
    """
    first = next(iter(frames.values()))
    axes = [c for c in first.columns if c.startswith("axis_") and first[c].nunique(dropna=False) > 1]
    if not axes:
        return None

    columns = min(3, len(axes))
    rows = math.ceil(len(axes) / columns)
    fig = make_subplots(rows=rows, cols=columns, subplot_titles=[a[5:] for a in axes])

    for i, axis in enumerate(axes):
        values = list(pd.unique(first[axis]))
        for k, (label, df) in enumerate(frames.items()):
            rate = [100.0 * float(df[df[axis] == v]["pass"].astype(bool).mean()) for v in values]
            trials = [int((df[axis] == v).sum()) for v in values]
            fig.add_trace(
                go.Scatter(x=[str(v) for v in values], y=rate, mode="lines+markers", name=label, legendgroup=label, showlegend=i == 0, line={"color": COLORS[k % len(COLORS)]}, customdata=trials, hovertemplate="%{x} : %{y:.0f} % of %{customdata} trials<extra>" + label + "</extra>"),
                row=i // columns + 1,
                col=i % columns + 1,
            )
        fig.update_yaxes(range=[-5, 105], title_text="pass rate (%)" if i % columns == 0 else None, row=i // columns + 1, col=i % columns + 1)

    fig.update_layout(height=320 * rows, title="Pass rate by axis", hovermode="x unified")

    return _save(fig=fig, out=out, name="pass_rate_by_axis")


def failure_reasons(out: Path, frames: dict[str, pd.DataFrame]) -> str | None:
    """
    Writes the number of failed trials for each reason and controller, and returns the link.
    """
    counts = {label: reason_of(df=df).loc[lambda s: s != ""].value_counts() for label, df in frames.items()}
    reasons = list(dict.fromkeys(r for c in counts.values() for r in c.index))
    if not reasons:
        return None

    fig = go.Figure()
    for k, reason in enumerate(reasons):
        fig.add_trace(go.Bar(x=list(frames), y=[int(counts[label].get(reason, 0)) for label in frames], name=reason, marker_color=COLORS[k % len(COLORS)]))

    fig.update_layout(barmode="stack", title="Failed trials by reason", yaxis_title="trials", height=420)

    return _save(fig=fig, out=out, name="failure_reasons")


def _timeline(trace: Path, row: pd.Series, title: str, path: Path) -> None:
    """
    Writes the timeline of one trial : height, roll and pitch, and the velocities against their commands.
    """
    data = np.load(trace, allow_pickle=False)
    odom, cmd = data["odom"], data["cmd_tx"].reshape(-1, 5)
    meta = json.loads(str(data["meta"])) if "meta" in data else {}
    t = odom[:, 0]
    roll, pitch, _ = euler(q=odom[:, 4:8])

    fig = make_subplots(rows=4, cols=1, shared_xaxes=True, vertical_spacing=0.04, subplot_titles=["base height (m)", "roll and pitch (rad)", "vx (m/s)", "yaw rate (rad/s)"])
    fig.add_trace(go.Scatter(x=t, y=odom[:, 3], name="height", line={"color": COLORS[0]}), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=roll, name="roll", line={"color": COLORS[1]}), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=pitch, name="pitch", line={"color": COLORS[2]}), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=odom[:, 8], name="vx", line={"color": COLORS[0]}), row=3, col=1)
    fig.add_trace(go.Scatter(x=t, y=odom[:, 13], name="wz", line={"color": COLORS[0]}, showlegend=False), row=4, col=1)
    if len(cmd):
        fig.add_trace(go.Scatter(x=cmd[:, 0], y=cmd[:, 1], name="command", line={"color": "gray", "dash": "dash"}), row=3, col=1)
        fig.add_trace(go.Scatter(x=cmd[:, 0], y=cmd[:, 3], name="command", line={"color": "gray", "dash": "dash"}, showlegend=False), row=4, col=1)

    # limits of the metrics
    fig.add_hline(y=DEFAULTS["fall_height"], line={"color": "red", "dash": "dot"}, row=1, col=1)
    for limit in (-DEFAULTS["rp_limit"], DEFAULTS["rp_limit"]):
        fig.add_hline(y=limit, line={"color": "red", "dash": "dot"}, row=2, col=1)

    # pushes and fall
    settle = float(meta.get("profile", {}).get("settle", 0.0))
    for push in meta.get("pushes", []):
        fig.add_vrect(x0=settle + push["at"], x1=settle + push["at"] + push["duration"], fillcolor="orange", opacity=0.25, line_width=0, row="all", col=1)
    fall = row.get("time_to_fall")
    if fall is not None and np.isfinite(fall):
        fig.add_vline(x=float(fall), line={"color": "red"}, row="all", col=1)

    fig.update_xaxes(title_text="time (s)", row=4, col=1)
    fig.update_layout(title=title, height=820, hovermode="x unified")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(path, include_plotlyjs="directory")


def _save(fig: go.Figure, out: Path, name: str) -> str:
    """
    Writes the figure in the plots folder and returns its link.
    """
    (out / "plots").mkdir(parents=True, exist_ok=True)
    fig.write_html(out / "plots" / f"{name}.html", include_plotlyjs="directory")

    return f"plots/{name}.html"
