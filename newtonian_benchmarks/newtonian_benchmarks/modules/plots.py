#pyright: reportAttributeAccessIssue=false
#pyright: reportCallIssue=false
#pyright: reportArgumentType=false
#pyright: reportGeneralTypeIssues=false

from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from newtonian_benchmarks.modules.plugins.metrics import DEFAULTS
from newtonian_benchmarks.modules.plugins.metrics.base import Context

# trials with plots
TRIALS = 3

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf"]

# the plots of a trial : file prefix and title
KINDS = (("command_tracking", "Command tracking"), ("base_states", "Base states"), ("control_inputs", "Control inputs"))


def reason_of(df: pd.DataFrame) -> pd.Series:
    """
    Returns the reason of failure of each trial, and an empty string for the trials that passed.
    """
    if "fail_reason" in df:
        return pd.Series(df["fail_reason"].fillna("").astype(str), index=df.index)

    fell = df["fell"].astype(bool) if "fell" in df else pd.Series(False, index=df.index)

    return pd.Series(np.where(df["pass"].astype(bool), "", np.where(fell, "fell", "other")), index=df.index)


def select_trials(frames: dict[str, pd.DataFrame], count: int = TRIALS) -> list[str]:
    """
    Returns the ids of the trials to plot : the first failed ones, then the ones with the largest velocity error.
    """
    ids: list[str] = []

    for df in frames.values():
        ids += [i for i in df[reason_of(df=df) != ""]["id"] if i not in ids]

    # the hardest trials of the rest
    for df in frames.values():
        if "rmse_ss_vx" in df:
            ids += [i for i in df.sort_values("rmse_ss_vx", ascending=False)["id"] if i not in ids][: max(0, count - len(ids))]

    return ids[:count]


def describe(df: pd.DataFrame, trial: str) -> str:
    """
    Returns what makes a trial different : its varied axes, the seed, and the suite.
    """
    row = df[df["id"] == trial].iloc[0]
    parts = [f"{c[5:]} {row[c]}" for c in df.columns if c.startswith("axis_") and df[c].nunique(dropna=False) > 1]
    for key in ("suite", "seed"):
        if key in df and df[key].nunique(dropna=False) > 1:
            parts.append(f"{key} {row[key]}")

    return ", ".join(parts) or f"trial {trial}"


def trial_label(df: pd.DataFrame, trial: str) -> str:
    """
    Returns the description of a trial with its id.
    """
    return f"{describe(df=df, trial=trial)} (id {trial})"


def slug(text: str) -> str:
    """
    Returns 'text' as a file name.
    """
    return re.sub(r"[^A-Za-z0-9.+_-]+", "_", text).strip("_")[:90]


def trial_plots(out: Path, frames: dict[str, pd.DataFrame], sources: dict[str, tuple[Path, str]], trials: list[str]) -> dict[str, list[tuple[str, str]]]:
    """
    Writes the tracking, base state, and control input plots of the trials, one line for each controller, and returns the title and image link of each.
    """
    links: dict[str, list[tuple[str, str]]] = {}

    for trial in trials:
        traces = {}
        for label, (folder, controller) in sources.items():
            path = folder / "traces" / controller / f"{trial}.npz"
            if path.exists():
                traces[label] = _load(path=path)
        if not traces:
            continue

        frame = next(df for df in frames.values() if (df["id"] == trial).any())
        what = describe(df=frame, trial=trial)
        name = slug(text=what.replace(" ", "-").replace(",-", "_")) or trial

        links[trial] = []
        for kind, title in KINDS:
            fig = {"command_tracking": _tracking, "base_states": _base, "control_inputs": _control}[kind](traces=traces, title=f"{title} : {what}")
            if fig is None:
                continue
            (out / "plots").mkdir(parents=True, exist_ok=True)
            fig.savefig(out / "plots" / f"{kind}_{name}.png", dpi=110)
            plt.close(fig)
            links[trial].append((title, f"plots/{kind}_{name}.png"))

    return links


def _load(path: Path) -> dict:
    """
    Returns the signals of a trial.
    """
    data = np.load(path, allow_pickle=False)
    meta = json.loads(str(data["meta"]))
    th = {**DEFAULTS, "cmd_period": None}
    ctx = Context.from_trial(odom=data["odom"], js=[], cmd_rx=data["cmd_rx"], profile=meta["profile"], pushes=meta["pushes"], timing=meta["timing"], rtf_target=meta["rtf"], th=th)
    joints = int(meta.get("joints", 0))

    return {
        "ctx": ctx,
        "odom": data["odom"],
        "joints": joints,
        "cmd_u": data["cmd_u"] if "cmd_u" in data else np.zeros((0, 1)),
        "js_u": data["js_u"] if "js_u" in data else np.zeros((0, 1)),
        "settle": float(meta["profile"].get("settle", 0.0)),
        "pushes": meta["pushes"],
    }


def _figure(rows: int, names: list[str], traces: dict, title: str) -> tuple[Figure, list[Axes]]:
    """
    Returns a figure of stacked panels with a shared time axis and the pushes marked.
    """
    fig, panels = plt.subplots(rows, 1, sharex=True, figsize=(9.0, 2.0 * rows + 1.0), squeeze=False)
    panels = [p[0] for p in panels]
    first = next(iter(traces.values()))

    for panel, name in zip(panels, names):
        panel.set_ylabel(name, fontsize=9)
        panel.grid(alpha=0.3)
        for push in first["pushes"]:
            panel.axvspan(first["settle"] + push["at"], first["settle"] + push["at"] + push["duration"], color="orange", alpha=0.25, linewidth=0)

    panels[-1].set_xlabel("time (s)")
    panels[0].set_title(title, fontsize=10)

    return fig, panels


def _finish(fig: Figure, panels: list[Axes], spans: list[float]) -> Figure:
    """
    Gives each panel a visible range of at least its span, and plain tick labels, so that a flat signal is not zoomed to its rounding noise.
    """
    for panel, span in zip(panels, spans):
        low, high = panel.get_ylim()
        if high - low < span:
            center = 0.5 * (low + high)
            panel.set_ylim(center - 0.5 * span, center + 0.5 * span)
        panel.ticklabel_format(axis="y", useOffset=False, style="plain")
    fig.tight_layout()

    return fig


def _marks(traces: dict, show_pushes: bool, show_falls: bool = True) -> list:
    """
    Returns the keys of the pushes and of the falls that the trial has.
    """
    marks: list = []
    if show_pushes and next(iter(traces.values()))["pushes"]:
        marks.append(Patch(color="orange", alpha=0.25, label="push"))
    if show_falls and any(t["ctx"].fell for t in traces.values()):
        marks.append(Line2D([], [], color="gray", linestyle=":", label="fall (controller color)"))

    return marks


def _keys(panel: Axes, traces: dict, extra: list | None = None, median: str = "") -> None:
    """
    Adds the legend of a panel : one key for each controller, then the keys of the other lines.
    """
    handles = [Line2D([], [], color=COLORS[k % len(COLORS)], label=f"{label} : {median}" if median else label) for k, label in enumerate(traces)] + (extra or [])
    panel.legend(handles=handles, fontsize=7, loc="upper right", ncol=min(len(handles), 4), framealpha=0.85)


def _tracking(traces: dict, title: str) -> Figure:
    """
    Returns the velocities and the yaw rate against their commands.
    """
    fig, panels = _figure(rows=3, names=["vx (m/s)", "vy (m/s)", "yaw rate (rad/s)"], traces=traces, title=title)
    first = next(iter(traces.values()))["ctx"]

    for i, key in enumerate((0, 1, 2)):
        panels[i].plot(first.t, first.cmd[:, key], color="gray", linestyle="--")
    for k, trace in enumerate(traces.values()):
        ctx = trace["ctx"]
        for i, signal in enumerate((ctx.v_h[:, 0], ctx.v_h[:, 1], ctx.wz)):
            panels[i].plot(ctx.t, signal, color=COLORS[k % len(COLORS)])
        if ctx.fell:
            for panel in panels:
                panel.axvline(ctx.t_fall, color=COLORS[k % len(COLORS)], linestyle=":")

    command = Line2D([], [], color="gray", linestyle="--", label="command")
    for i, panel in enumerate(panels):
        _keys(panel=panel, traces=traces, extra=[command] + _marks(traces=traces, show_pushes=i == 0))

    return _finish(fig=fig, panels=panels, spans=[0.2, 0.2, 0.4])


def _base(traces: dict, title: str) -> Figure:
    """
    Returns the height, the attitude, and the angular speed of the base.
    """
    fig, panels = _figure(rows=4, names=["height (m)", "roll (rad)", "pitch (rad)", "angular speed (rad/s)"], traces=traces, title=title)
    panels[0].axhline(DEFAULTS["fall_height"], color="red", linestyle=":")
    for panel in panels[1:3]:
        for limit in (-DEFAULTS["rp_limit"], DEFAULTS["rp_limit"]):
            panel.axhline(limit, color="red", linestyle=":")

    for k, trace in enumerate(traces.values()):
        ctx, color = trace["ctx"], COLORS[k % len(COLORS)]
        panels[0].plot(ctx.t, ctx.pos[:, 2], color=color)
        panels[1].plot(ctx.t, ctx.roll, color=color)
        panels[2].plot(ctx.t, ctx.pitch, color=color)
        panels[3].plot(ctx.t, np.linalg.norm(trace["odom"][:, 11:14], axis=1), color=color)
        if ctx.fell:
            for panel in panels:
                panel.axvline(ctx.t_fall, color=color, linestyle=":")

    limits = [[Line2D([], [], color="red", linestyle=":", label="fall height")], [Line2D([], [], color="red", linestyle=":", label="roll limit")], [Line2D([], [], color="red", linestyle=":", label="pitch limit")], []]
    for i, panel in enumerate(panels):
        _keys(panel=panel, traces=traces, extra=limits[i] + _marks(traces=traces, show_pushes=i == 0))

    return _finish(fig=fig, panels=panels, spans=[0.05, 0.1, 0.1, 1.0])


def _control(traces: dict, title: str) -> Figure | None:
    """
    Returns the commanded joint positions, the applied joint efforts, and the period of the commands.
    """
    if not any(len(t["cmd_u"]) > 1 and t["joints"] for t in traces.values()):
        return None

    fig, panels = _figure(rows=3, names=["commanded position change (rad)", "joint effort (N m)", "command period (ms)"], traces=traces, title=title)

    for k, trace in enumerate(traces.values()):
        color, n = COLORS[k % len(COLORS)], trace["joints"]
        cmd, js = trace["cmd_u"], trace["js_u"]
        if len(cmd) > 1:
            position = cmd[:, 1 : 1 + n] - cmd[0, 1 : 1 + n]
            panels[0].plot(cmd[:, 0], np.median(position, axis=1), color=color)
            panels[0].fill_between(cmd[:, 0], position.min(axis=1), position.max(axis=1), color=color, alpha=0.2, linewidth=0)
            panels[2].plot(cmd[1:, 0], np.round(1000.0 * np.diff(cmd[:, 0]), 3), color=color)
        if len(js) > 1:
            effort = np.abs(js[:, 1 + n : 1 + 2 * n])
            panels[1].plot(js[:, 0], np.sqrt(np.mean(effort**2, axis=1)), color=color)
            panels[1].plot(js[:, 0], effort.max(axis=1), color=color, linestyle=":")

    _keys(panel=panels[0], traces=traces, extra=[Patch(color="gray", alpha=0.3, label="lowest to highest joint (band)")] + _marks(traces=traces, show_pushes=True, show_falls=False), median="median of the joints")
    _keys(panel=panels[1], traces=traces, extra=[Line2D([], [], color="gray", linestyle=":", label="largest joint (dotted)")], median="RMS of the joints")
    _keys(panel=panels[2], traces=traces)
    panels[2].set_ylim(bottom=0.0)

    return _finish(fig=fig, panels=panels, spans=[0.2, 2.0, 1.0])
