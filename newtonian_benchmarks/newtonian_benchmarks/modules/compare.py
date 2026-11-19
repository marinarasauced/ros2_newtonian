#pyright: reportAttributeAccessIssue=false
#pyright: reportCallIssue=false
#pyright: reportArgumentType=false
#pyright: reportGeneralTypeIssues=false

from __future__ import annotations

from typing import Any

import itertools
import json
from pathlib import Path

import pandas as pd

from newtonian_benchmarks.modules import plots
from newtonian_benchmarks.modules.report import GRID_CELLS, LEGEND, axes_of, cell, grid, grid_title, markdown_table, number, percent, summary


# trials listed in the table of differences
DIFFERENCES = 30

# columns of the scorecard : key, title, format, and whether a higher value is better
COLUMNS = [
    ("pass_rate", "Pass", percent, True),
    ("survival_rate", "Survival", percent, True),
    ("recovery_rate", "Recovery", percent, True),
    ("rt_ok_rate", "Real-time ok", percent, True),
    ("rmse_ss_vx_median", "Error vx", number, False),
    ("rmse_ss_wz_median", "Error wz", number, False),
    ("rp_exceed_frac_median", "Roll/pitch over", percent, False),
    ("recovery_time_median", "Recovery time (s)", number, False),
]


def write(dirs: list[Path], out: Path) -> Path:
    """
    Writes compare.md for the controllers of the results folders 'dirs'.
    """
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)

    frames = _frames(dirs=dirs)
    labels = list(frames)
    if len(labels) < 2:
        raise ValueError("a comparison needs the results of two or more controllers")

    runs = {label: len(df) for label, df in frames.items()}

    # trials that every controller ran
    common = list(frames[labels[0]].index)
    for df in frames.values():
        common = [i for i in common if i in df.index]
    if not common:
        raise ValueError("the results have no trial in common")

    frames = {label: df.loc[common] for label, df in frames.items()}
    first = frames[labels[0]].reset_index()
    parts = ["# Controller comparison", LEGEND, f"{len(common)} trials in common · " + " · ".join(f"{label} : {runs[label]} run" for label in labels)]

    # scorecard : the best value of a column is in bold
    stats = {label: summary(df=df.reset_index()) for label, df in frames.items()}
    parts += ["## Scorecard", _scorecard(stats=stats, labels=labels)]

    # suites
    if "suite" in first and first["suite"].nunique() > 1:
        rows = [[name, *[cell(passed=frames[label][frames[label]["suite"] == name]["pass"]) for label in labels]] for name in pd.unique(first["suite"])]
        parts += ["## Suites", markdown_table(header=["Suite", *labels], rows=rows)]

    # axes
    for a in axes_of(df=first):
        rows = [[str(v), *[cell(passed=frames[label][first[a].to_numpy() == v]["pass"]) for label in labels]] for v in pd.unique(first[a])]
        parts += [f"## {a[5:]}", markdown_table(header=[a[5:], *labels], rows=rows)]

    # one grid for each pair of axes and each controller
    axes = axes_of(df=first)
    for a, b in itertools.combinations(axes, 2):
        if first[a].nunique() * first[b].nunique() > GRID_CELLS:
            continue

        parts.append(f"## {grid_title(df=first, a=a, b=b)}")
        for label in labels:
            parts += [f"**{label}**", grid(df=frames[label].reset_index(), a=a, b=b)]

    # interactive plots
    links = [("Pass rate by axis", plots.pass_rate_by_axis(out=out, frames=frames)), ("Failed trials by reason", plots.failure_reasons(out=out, frames=frames))]
    links = [(label, link) for label, link in links if link]
    if links:
        parts += ["## Plots", "\n".join(f"- [{label}]({link})" for label, link in links) + "\n- Timelines of failed trials : " + ", ".join(f"[{label}](report_{label}.md)" for label in labels)]

    # trials where the controllers differ
    passed = pd.DataFrame({label: df["pass"].astype(bool) for label, df in frames.items()})
    differ = passed[passed.nunique(axis=1) > 1]
    if len(differ):
        axes = [c for c in first.columns if c.startswith("axis_") and first[c].nunique(dropna=False) > 1]
        info = first.set_index("id")
        rows = []
        for tid in differ.index[:DIFFERENCES]:
            rows.append([tid, *[str(info.loc[tid, c]) for c in axes], *[_result(frames[label].loc[tid]) for label in labels]])
        parts += [f"## Differences ({len(differ)})", markdown_table(header=["id", *[c[5:] for c in axes], *labels], rows=rows)]
        if len(differ) > DIFFERENCES:
            parts.append(f"{len(differ) - DIFFERENCES} more.")
    else:
        parts.append("## Differences\n\nNone : the controllers pass and fail the same trials.")

    path = out / "compare.md"
    path.write_text("\n\n".join(parts) + "\n")
    (out / "compare.json").write_text(json.dumps(stats, indent=1))

    return path


def _frames(dirs: list[Path]) -> dict[str, pd.DataFrame]:
    """
    Returns the trials of each controller of the results folders, by trial id.
    """
    frames: dict[str, pd.DataFrame] = {}

    for d in dirs:
        df = pd.read_csv(d / "results.csv")
        parts = {str(name): part for name, part in df.groupby("controller", sort=False)} if "controller" in df else {_name(folder=d): df}

        for label, part in parts.items():
            frames[label if label not in frames else f"{label} ({d.name})"] = part.set_index("id")

    return frames


def _name(folder: Path) -> str:
    """
    Returns the controller of a results folder without a controller column.
    """
    saved = folder / "suite.json"
    if saved.exists():
        data = json.loads(saved.read_text())
        if data.get("controllers"):
            return str(data["controllers"][0]["name"])
        if data.get("controller"):
            return str(data["controller"]["name"])

    return folder.name


def _scorecard(stats: dict[str, dict[str, Any]], labels: list[str]) -> str:
    """
    Returns the scorecard of the controllers as a table.
    """
    best = {}
    for key, _, _, higher in COLUMNS:
        values = {label: stats[label].get(key) for label in labels if stats[label].get(key) is not None and not pd.isna(stats[label].get(key))}
        if len(set(values.values())) > 1:
            best[key] = (max if higher else min)(values.values())

    rows = []
    for label in labels:
        row = [label]
        for key, _, fmt, _ in COLUMNS:
            text = fmt(x=stats[label].get(key))
            row.append(f"**{text}**" if key in best and stats[label].get(key) == best[key] else text)
        rows.append(row)

    return markdown_table(header=["Controller", *[title for _, title, _, _ in COLUMNS]], rows=rows)


def _result(row: pd.Series) -> str:
    """
    Returns the result of one trial for the table of differences.
    """
    if str(row.get("fell")) == "True":
        return "❌ fell"

    return f"{'✅' if bool(row['pass']) else '❌'} vx {number(x=row.get('rmse_ss_vx'), digits=2)}"
