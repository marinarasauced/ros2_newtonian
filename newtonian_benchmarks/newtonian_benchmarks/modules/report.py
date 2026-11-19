#pyright: reportAttributeAccessIssue=false
#pyright: reportCallIssue=false
#pyright: reportOperatorIssue=false
#pyright: reportArgumentType=false
#pyright: reportGeneralTypeIssues=false

from __future__ import annotations

from typing import Any

import itertools
import json
from pathlib import Path

import pandas as pd


MEDIANS = ("rmse_ss_vx", "rmse_ss_vy", "rmse_ss_wz", "rp_exceed_frac", "recovery_time", "cmd_age_p99", "sim_rtf")

# grids with more cells than this are skipped
GRID_CELLS = 60

# colors of the pass rates
LEGEND = "Pass rate : 🟥 below 25 % · 🟧 below 50 % · 🟨 below 90 % · 🟩 90 % or more"

# trials listed in the table of failures
FAILURES = 20


def write(out: Path) -> Path:
    """
    Writes report.md and summary.json from results.csv, and compare.md for more than one controller.
    """
    out = Path(out)
    df = pd.read_csv(out / "results.csv")
    names = list(pd.unique(df["controller"])) if "controller" in df else [None]

    # one report for each controller
    results = {}
    for name in names:
        part = df if name is None else df[df["controller"] == name]
        path = out / ("report.md" if len(names) == 1 else f"report_{name}.md")
        results[name] = _write_one(out=out, df=part, controller=name, path=path)

    (out / "summary.json").write_text(json.dumps(results[names[0]] if len(names) == 1 else results, indent=1))

    # the controllers side by side
    if len(names) > 1:
        from newtonian_benchmarks.modules import compare

        return compare.write(dirs=[out], out=out)

    return out / "report.md"


def _write_one(out: Path, df: pd.DataFrame, controller: str | None, path: Path) -> dict[str, Any]:
    """
    Writes the report of one controller and returns its numbers.
    """
    s = summary(df=df)
    result: dict[str, Any] = {"all": s}
    parts = [f"# Benchmark {out.name}", _headline(df=df, controller=controller), LEGEND, "## Scorecard", scorecard(s=s)]

    # one section for each suite
    suites = list(dict.fromkeys(df["suite"].dropna())) if "suite" in df else []
    if len(suites) > 1:
        result["suites"] = {name: summary(df=df[df["suite"] == name]) for name in suites}
        parts += ["## Suites", markdown_table(header=["Suite", "Trials", "Pass", "Falls", "Error vx", "Error wz", "Roll/pitch over", "Recovery"], rows=[[name, *_row(df=df[df["suite"] == name])] for name in suites])]

        for name in suites:
            parts.append(f"## {name}")
            _section(df=df[df["suite"] == name], parts=parts)
    else:
        _section(df=df, parts=parts)

    path.write_text("\n\n".join(parts) + "\n")

    return result


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


def scorecard(s: dict[str, Any]) -> str:
    """
    Returns the scorecard numbers as a table.
    """
    rows = [
        ["Trials", f"{s['trials']} ({s['valid']} valid)"],
        ["Pass", percent(x=s.get("pass_rate"))],
        ["Survival", percent(x=s.get("survival_rate"))],
        ["Recovery after a push", percent(x=s.get("recovery_rate"))],
        ["Real-time ok", percent(x=s.get("rt_ok_rate"))],
        ["Error vx, median (m/s)", number(x=s.get("rmse_ss_vx_median"))],
        ["Error wz, median (rad/s)", number(x=s.get("rmse_ss_wz_median"))],
        ["Roll/pitch over the limit, median", percent(x=s.get("rp_exceed_frac_median"))],
        ["Recovery time, median (s)", number(x=s.get("recovery_time_median"))],
        ["Simulator speed, median", number(x=s.get("sim_rtf_median"), digits=2)],
    ]

    return markdown_table(header=["Metric", "Value"], rows=rows)


def markdown_table(header: list[str], rows: list[list[str]]) -> str:
    """
    Returns a markdown table.
    """
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(str(c).replace("|", "/") for c in row) + " |" for row in rows]

    return "\n".join(lines)


def percent(x: float | None) -> str:
    """
    Returns a share as a percent.
    """
    return "-" if x is None or pd.isna(x) else f"{100.0 * x:.0f} %"


def number(x: float | None, digits: int = 3) -> str:
    """
    Returns a number, or a dash for a missing value.
    """
    return "-" if x is None or pd.isna(x) else f"{x:.{digits}f}"


def cell(passed: pd.Series) -> str:
    """
    Returns the pass rate of a set of trials with a color.
    """
    if len(passed) == 0:
        return "-"

    rate = float(passed.astype(bool).mean())

    return f"{'🟥' if rate < 0.25 else '🟧' if rate < 0.5 else '🟨' if rate < 0.9 else '🟩'} {100.0 * rate:.0f} %"


def grid(df: pd.DataFrame, a: str, b: str) -> str:
    """
    Returns the pass rate for each pair of values of the axes 'a' and 'b' as a table.
    """
    cols = list(pd.unique(df[b]))
    rows = [[str(r), *[cell(passed=df[(df[a] == r) & (df[b] == c)]["pass"]) for c in cols]] for r in pd.unique(df[a])]

    return markdown_table(header=[f"{a[5:]} \\ {b[5:]}", *map(str, cols)], rows=rows)


def grid_title(df: pd.DataFrame, a: str, b: str) -> str:
    """
    Returns the title of the grid of the axes 'a' and 'b', with the axes that are averaged.
    """
    others = [c[5:] for c in axes_of(df=df) if c not in (a, b)]

    return f"{a[5:]} × {b[5:]}" + (f" (averaged over {', '.join(others)})" if others else "")


def axes_of(df: pd.DataFrame) -> list[str]:
    """
    Returns the axes with more than one value.
    """
    return [c for c in df.columns if c.startswith("axis_") and df[c].nunique(dropna=False) > 1]


def _headline(df: pd.DataFrame, controller: str | None) -> str:
    """
    Returns the controller and the time of the trials.
    """
    names = [f"controller {controller}"] if controller else []
    if "wall_s" in df:
        names.append(f"{pd.to_numeric(df['wall_s'], errors='coerce').sum() / 60.0:.1f} min of trials")

    return " · ".join(names)


def _row(df: pd.DataFrame) -> list[str]:
    """
    Returns the table cells of a set of trials.
    """
    ok = df[df.get("valid", False) == True]  # noqa: E712
    median = lambda k: pd.to_numeric(ok[k], errors="coerce").median() if k in ok and len(ok) else float("nan")  # noqa: E731
    fell = int(df["fell"].astype(bool).sum()) if "fell" in df else 0
    recovered = "-"
    if "recovered" in ok and "pushes" in ok:
        pushed = ok[ok["pushes"] > 0]
        recovered = percent(x=float(pushed["recovered"].astype(bool).mean())) if len(pushed) else "-"

    return [str(len(df)), cell(passed=df["pass"]), str(fell), number(x=median("rmse_ss_vx")), number(x=median("rmse_ss_wz")), percent(x=median("rp_exceed_frac")), recovered]


def _section(df: pd.DataFrame, parts: list[str]) -> None:
    """
    Adds the tables of a set of trials : one for each axis, one grid for each pair of axes, and the failures.
    """
    axes = axes_of(df=df)

    for a in axes:
        rows = [[str(v), *_row(df=df[df[a] == v])] for v in pd.unique(df[a])]
        parts += [f"### {a[5:]}", markdown_table(header=[a[5:], "Trials", "Pass", "Falls", "Error vx", "Error wz", "Roll/pitch over", "Recovery"], rows=rows)]

    for a, b in itertools.combinations(axes, 2):
        if df[a].nunique() * df[b].nunique() > GRID_CELLS:
            continue

        parts += [f"### {grid_title(df=df, a=a, b=b)}", grid(df=df, a=a, b=b)]

    failed = df[~df["pass"].astype(bool)]
    if len(failed):
        columns = [c for c in axes]
        rows = [[r["id"], *[str(r[c]) for c in columns], "fell" if str(r.get("fell")) == "True" else str(r.get("status", "ok")).replace("ok", "limits"), number(x=r.get("rmse_ss_vx")), number(x=r.get("rmse_ss_wz"))] for _, r in failed.head(FAILURES).iterrows()]
        parts += [f"### Failed trials ({len(failed)})", markdown_table(header=["id", *[c[5:] for c in columns], "Result", "Error vx", "Error wz"], rows=rows)]
        if len(failed) > FAILURES:
            parts.append(f"{len(failed) - FAILURES} more in results.csv.")
