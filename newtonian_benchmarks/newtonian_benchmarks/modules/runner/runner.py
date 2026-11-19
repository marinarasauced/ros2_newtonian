
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import csv
import hashlib
import json
import os
import time
from pathlib import Path

import numpy as np

from newtonian_benchmarks.modules.config import apply_robot, defaults, load_yaml, scene
from newtonian_benchmarks.modules.hooks import find_pushes
from newtonian_benchmarks.modules.plugins import metrics
from newtonian_benchmarks.modules.runner.session import Session
from newtonian_benchmarks.modules.sweep import expand_run, scene_key, write_scene
from newtonian_benchmarks.modules.units import resolve_terrain

if TYPE_CHECKING:
    from newtonian_benchmarks.node import BenchmarkNode


def format_duration(seconds: float) -> str:
    """
    Returns 'seconds' as h, min, and s.
    """
    seconds = int(round(seconds))
    hours, rest = divmod(seconds, 3600)
    minutes, seconds = divmod(rest, 60)

    if hours:
        return f"{hours} h {minutes:02d} min"
    if minutes:
        return f"{minutes} min {seconds:02d} s"

    return f"{seconds} s"


class BenchmarkRunner:
    """
    Runs the trials of a config file and writes the results.
    """

    def __init__(self, config: dict[str, Any], controllers: list[dict[str, Any]], limits: dict[str, list[float]] | None, profile: dict[str, Any], out: Path, domain: int) -> None:
        """
        Stores the config, the controllers, the limits of the commands, the robot, the output folder, and the ROS domain.
        """
        self.config = config
        self.suite = defaults(run=config)
        self.controllers = controllers
        self.limits = limits
        self.profile = profile
        self.out = Path(out)
        self.domain = int(domain)

    @classmethod
    def from_yaml(cls, path: str, out: Path, domain: int = 60, rtf: float | None = None, only_controllers: list[str] | None = None) -> BenchmarkRunner:
        """
        Returns the runner of the config file 'path'.
        """
        run = load_yaml(path_or_name=path)
        profile = run.pop("robot", None)
        controllers = run.pop("controllers", None)
        single = run.pop("controller", None)
        if controllers is None and single is not None:
            controllers = [single]
        if not isinstance(profile, dict) or not isinstance(controllers, list) or not controllers or not all(isinstance(c, dict) and "name" in c for c in controllers):
            raise ValueError(f"{run['_path']} needs a 'robot' block and a 'controllers' list (or one 'controller' block) with a name for each")

        names = [c["name"] for c in controllers]
        if len(set(names)) != len(names):
            raise ValueError(f"controller names must be different : {names}")

        # the limits of the commands are the ones of the run, so that every controller gets the same commands
        limits = run.pop("limits", None) or controllers[0].get("limits")

        if only_controllers:
            missing = [n for n in only_controllers if n not in names]
            if missing:
                raise ValueError(f"no controller named {missing} in {run['_path']} : {names}")
            controllers = [c for c in controllers if c["name"] in only_controllers]

        if rtf is not None:
            run["rtf"] = rtf

        return cls(config=apply_robot(run=run, profile=profile), controllers=controllers, limits=limits, profile=profile, out=out, domain=domain)

    def trials(self, only: list[str] | None = None, limit: int | None = None) -> list[dict[str, Any]]:
        """
        Returns the trials of the suites.
        """
        trials = expand_run(run=self.config, limits=self.limits, only=only)

        # terrain keys in units of the nominal height of this robot
        for t in trials:
            if "terrain" in t["params"]:
                t["params"]["terrain"] = resolve_terrain(terrain=t["params"]["terrain"], height=float(self.profile["height"]))

        return trials[:limit] if limit else trials

    def run(self, node: BenchmarkNode, only: list[str] | None = None, limit: int | None = None) -> None:
        """
        Runs the trials and writes results.csv.
        """
        trials = self.trials(only=only, limit=limit)
        self.write_suite(trials=trials)

        t0 = time.monotonic()
        self._execute(node=node, trials=trials)
        rows = self.write_results(out=self.out)

        node.get_logger().info(f"{len(rows)} trials in {format_duration(seconds=time.monotonic() - t0)}, {sum(1 for r in rows if r.get('pass'))} passed -> {self.out}/results.csv")

    def write_suite(self, trials: list[dict[str, Any]]) -> None:
        """
        Writes suite.json before the trials run.
        """
        self.out.mkdir(parents=True, exist_ok=True)
        saved = {
            "suite": self.suite,
            "run": self.config,
            "controllers": self.controllers,
            "limits": self.limits,
            "trials": len(trials),
            "suites": sorted({t["suite"] for t in trials}),
        }
        (self.out / "suite.json").write_text(json.dumps(saved, indent=1, default=str))

    def write_report(self) -> None:
        """
        Writes report.md from results.csv.
        """
        from newtonian_benchmarks.modules import report

        report.write(out=self.out)

    @staticmethod
    def write_results(out: Path, rows: list[dict[str, Any]] | None = None) -> list[dict[str, Any]]:
        """
        Writes results.csv and returns the rows.
        """
        if rows is None:
            rows = BenchmarkRunner._read_rows(out=out)

        keys = sorted({k for r in rows for k in r})
        with (out / "results.csv").open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)

        return rows

    @staticmethod
    def reanalyze(out: Path) -> None:
        """
        Computes the metrics of the traces in 'out' again.
        """
        from newtonian_benchmarks.modules import report

        saved = json.loads((out / "suite.json").read_text())
        controllers = {c["name"]: c for c in saved["controllers"]}
        first = next(iter(controllers))
        suite = saved["suite"]
        trials = {t["id"]: t for t in expand_run(run=saved.get("run", suite), limits=saved.get("limits"))}

        rows = []
        for row in BenchmarkRunner._read_rows(out=out):
            tid = row["id"]
            ctl = controllers.get(row.get("controller", first), controllers[first])
            f = out / "traces" / ctl["name"] / f"{tid}.npz"

            # a trial without trace keeps its row
            if not f.exists() or tid not in trials:
                rows.append(row)
                continue

            d = np.load(f, allow_pickle=False)
            cfg = trials[tid].get("cfg", suite)
            params = {**cfg.get("trial", {}), **trials[tid]["params"]}

            # profile, pushes and timing of the trial
            if "meta" in d:
                meta = json.loads(str(d["meta"]))
                profile, pushes, timing, rtf, duration = meta["profile"], meta["pushes"], meta["timing"], meta["rtf"], meta["duration"]
            else:
                profile = json.loads(str(d["profile"]))
                pushes = [{"at": float(p.get("at", 3.0)), "duration": float(p.get("duration", 0.2))} for p in find_pushes(params=params)]
                timing = {"rtf": row.get("sim_rtf", 0.0), "frames": row.get("sim_frames", 0), "overruns": row.get("sim_overruns", 0)}
                rtf = float(params.get("rtf", suite.get("rtf", 1.0)))
                duration = float(params.get("duration", 8.0))

            # metrics and verdict
            thresholds = {**cfg.get("thresholds", {}), "cmd_period": 1.0 / float(ctl["rate_hz"]) if ctl.get("rate_hz") else None}
            m = metrics.evaluate(odom=d["odom"], js=[], cmd_rx=d["cmd_rx"], profile=profile, duration=duration, pushes=pushes, timing=timing, rtf_target=rtf, thresholds=thresholds)
            m["pass"] = bool(metrics.is_passed(m=m, rule=cfg.get("pass", {}))) and row.get("status") == "ok"
            m["fail_reason"] = "" if m["pass"] else metrics.failure_reason(m={**m, "status": row.get("status", "ok")}, rule=cfg.get("pass", {})) or "other"

            # values that only the run knows
            for k in ("status", "wall_s", "clock_jumps", "rtf_used", "mech_power", "effort_max"):
                if k in row and k not in m:
                    m[k] = row[k]

            rows.append({k: v for k, v in row.items() if k.startswith("axis_") or k in ("id", "seed", "suite", "controller")} | m)

        BenchmarkRunner.write_results(out=out, rows=rows)
        report.write(out=out)
        print(f"reanalyzed {len(rows)} trials, {sum(1 for r in rows if r.get('pass'))} passed")

    def _execute(self, node: BenchmarkNode, trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """
        Runs the trials with each controller and returns the rows.
        """
        suite, out = self.suite, self.out
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(self.domain)
        base = scene(profile=self.profile)

        # trials of the same scene
        groups: dict[str, list[dict[str, Any]]] = {}
        for t in trials:
            groups.setdefault(scene_key(params=t["params"], suite=t.get("cfg", suite)), []).append(t)

        logger = node.get_logger()
        total = len(trials) * len(self.controllers)
        start = time.monotonic()
        rows: list[dict[str, Any]] = []
        (out / "traces").mkdir(parents=True, exist_ok=True)
        sink = (out / "results.jsonl").open("w")

        def record(t: dict[str, Any], m: dict[str, Any], controller: str) -> None:
            """
            Stores the row of a trial and logs the progress.
            """
            row = self._row(t=t, m=m, controller=controller)
            rows.append(row)
            sink.write(json.dumps(row) + "\n")
            sink.flush()

            elapsed = time.monotonic() - start
            remaining = elapsed / len(rows) * (total - len(rows))
            logger.info(
                f"[{len(rows)}/{total}, {100 * len(rows) // total} %] {row['suite']} {controller} {t['id']} {t['axes']} "
                f"pass={row['pass']} fell={row.get('fell')} status={row['status']} "
                f"| elapsed {format_duration(seconds=elapsed)}, remaining about {format_duration(seconds=remaining)}"
            )

        try:
            for gi, (key, items) in enumerate(groups.items()):
                cfg = write_scene(base=base, key=key, out=out / "scenes" / f"{hashlib.sha1(key.encode()).hexdigest()[:8]}.yaml")
                label = f"scene {gi + 1}/{len(groups)}"

                # simulator of the scene
                logger.info(f"{label} : {len(items)} trials, starting the simulator")
                try:
                    session = Session(suite=suite, scene_cfg=cfg, out=out, env=env, node=node, tag=f"g{gi}")
                except Exception as e:
                    logger.error(f"{label} failed : {e}")
                    for ctl in self.controllers:
                        for t in items:
                            record(t=t, m={"status": "scene_failed", "pass": False, "valid": False, "error": str(e)}, controller=ctl["name"])
                    continue

                try:
                    for ctl in self.controllers:
                        name = ctl["name"]

                        # controller of the scene
                        logger.info(f"{label}, controller {name} : starting")
                        try:
                            session.start(ctl=ctl)
                        except Exception as e:
                            logger.error(f"{label}, controller {name} failed : {e}")
                            for t in items:
                                record(t=t, m={"status": "controller_failed", "pass": False, "valid": False, "error": str(e)}, controller=name)
                            continue

                        # trials
                        for t in items:
                            try:
                                m = session.run(t=t)
                            except Exception as e:
                                m = {"status": "error", "pass": False, "valid": False, "error": f"{type(e).__name__}: {e}"}

                                # a failed trial may leave the processes in a bad state : start them again
                                try:
                                    session.close()
                                    session = Session(suite=suite, scene_cfg=cfg, out=out, env=env, node=node, tag=f"g{gi}r")
                                    session.start(ctl=ctl)
                                except Exception as e2:
                                    m["error"] += f" / restart failed: {e2}"

                            record(t=t, m=m, controller=name)
                finally:
                    session.close()
        finally:
            sink.close()

        return rows

    def _row(self, t: dict[str, Any], m: dict[str, Any], controller: str) -> dict[str, Any]:
        """
        Returns the row of trial 't' of 'controller'.
        """
        axes = {f"axis_{k}": v for k, v in t["axes"].items()}

        return {"id": t["id"], "controller": controller, "suite": t.get("suite", self.suite.get("name")), "seed": t["seed"], **axes, **m}

    @staticmethod
    def _read_rows(out: Path) -> list[dict[str, Any]]:
        """
        Returns the rows of the results files in 'out'.
        """
        return [json.loads(line) for f in sorted(out.glob("results*.jsonl")) for line in f.read_text().splitlines() if line.strip()]
