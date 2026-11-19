
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import json
import time
from pathlib import Path

import numpy as np
import yaml

from newtonian_benchmarks.modules.hooks import find_pushes, to_request
from newtonian_benchmarks.modules.plugins import metrics
from newtonian_benchmarks.modules.runner.processes import Process, expand_environment
from newtonian_benchmarks.modules.sweep import COMMAND_KEYS
from newtonian_benchmarks.modules.units import resolve_parameters

if TYPE_CHECKING:
    from newtonian_benchmarks.node import BenchmarkNode


class Session:
    """
    Simulator and controller of one scene.
    """

    def __init__(self, suite: dict[str, Any], ctl: dict[str, Any], scene_cfg: Path, out: Path, env: dict[str, str], node: BenchmarkNode, tag: str) -> None:
        """
        Starts the simulator and the controller.
        """
        self.suite = suite
        self.ctl = ctl
        self.out = out
        self.env = env
        self.node = node
        self.tag = tag
        self.ctl_proc: Process | None = None

        # simulator: its process, and the sensors that may get latency (not the ground truth)
        cmd = ["ros2", "run", "newtonian", "sim", "--ros-args", "--params-file", str(scene_cfg)]
        self.sim = Process(cmd, out / "logs" / f"{tag}_sim.log", env)
        params = yaml.safe_load(scene_cfg.read_text())["newtonian"]["ros__parameters"]
        self.sensors = [name for name in params.get("sensors", {}) if not name.endswith("_gt")]

        # facts about the scene
        info = node.configure({"query": "info"}, timeout=90.0)
        self.frame_rate = float(info["frame_rate"])
        self.mass = float(info["mass"])
        self.gravity = float(info["gravity"])
        self.height = float(suite["trial"]["height"])

        # controller
        self._start_controller()

        # real-time factor: the one of the file, or the highest one at which both sides keep up
        if suite.get("rtf", 1.0) == "auto":
            self.rtf_used = self._calibrate(candidates=suite.get("rtf_candidates", [1.0, 1.25, 1.5, 2.0, 3.0, 4.0]))
            print(f"[{tag}] real-time factor {self.rtf_used}; probes {json.dumps(self.calibration)}", flush=True)
            (out / f"calibration_{tag}.json").write_text(json.dumps({"rtf": self.rtf_used, "probes": self.calibration}, indent=1))
        else:
            self.rtf_used = float(suite.get("rtf", 1.0))

    def run(self, t: dict[str, Any]) -> dict[str, Any]:
        """
        Runs one trial and returns its metrics.
        """
        node = self.node
        cfg = t.get("cfg", self.suite)
        params = resolve_parameters(params={**cfg.get("trial", {}), **t["params"]}, mass=self.mass, height=self.height, gravity=self.gravity)
        settle = float(params.get("settle", 1.0))
        duration = float(params.get("duration", 8.0))
        rtf = float(params.get("rtf", self.rtf_used))

        # a controller without a reset starts again
        if self.ctl.get("reset", "none") == "restart":
            self._start_controller()

        # scene: reset, physics, pushes, latency, seed; the timing starts again
        req = to_request(params=params, sensors=self.sensors)
        req["seed"] = int(t["seed"])
        req["timing"] = True
        req.setdefault("rtf", rtf)
        node.configure(req)

        # the simulator warms up; the timing of the warm-up is discarded
        self._wait_warmup()
        node.configure({"timing": True})

        # command profile and recording
        profile = {k: params[k] for k in COMMAND_KEYS if k in params}
        profile["settle"] = settle
        profile.setdefault("height", self.height)
        node.start(profile=profile, duration=duration)

        # the node ends the trial at the duration, or after a fall; the wall time is limited
        wall0 = time.monotonic()
        limit = duration / rtf * 3.0 + 30.0
        status = "ok"
        while not node.done:
            time.sleep(0.02)

            if time.monotonic() - wall0 > limit:
                status = "timeout"
                break
            if not self.sim.is_alive():
                status = "sim_died"
                break
            if self.ctl_proc is not None and not self.ctl_proc.is_alive():
                status = "controller_died"
                break
        node.stop()

        # records and timing
        timing = node.configure({"timing": True}).get("timing", {}) if status == "ok" else {}
        odom = np.array(node.odom, dtype=float).reshape(-1, 14)
        cmd_rx = np.array(node.cmd_rx, dtype=float).reshape(-1, 2)
        pushes = [{"at": float(p.get("at", 3.0)), "duration": float(p.get("duration", 0.2))} for p in find_pushes(params=params)]

        # metrics and verdict
        thresholds = {**cfg.get("thresholds", {}), "cmd_period": 1.0 / float(self.ctl["rate_hz"]) if self.ctl.get("rate_hz") else None}
        m = metrics.evaluate(odom=odom, js=node.js, cmd_rx=cmd_rx, profile=profile, duration=duration, pushes=pushes, timing=timing, rtf_target=rtf, thresholds=thresholds)
        m["status"] = status
        m["rtf_used"] = rtf
        m["pass"] = bool(metrics.is_passed(m=m, rule=cfg.get("pass", {}))) and status == "ok"
        m["wall_s"] = time.monotonic() - wall0
        m["clock_jumps"] = node.clock_jumped_back

        # trace, for 'reanalyze'
        meta = {"profile": profile, "pushes": pushes, "timing": timing, "rtf": rtf, "duration": duration}
        np.savez_compressed(self.out / "traces" / f"{t['id']}.npz", odom=odom, cmd_rx=cmd_rx, cmd_tx=np.array(node.cmd_tx), meta=json.dumps(meta))

        return m

    def close(self) -> None:
        """
        Stops the controller and the simulator.
        """
        if self.ctl_proc is not None:
            self.ctl_proc.stop()
        self.sim.stop()

    def _stand(self) -> dict[str, Any]:
        """
        Returns the command profile of a robot that stands.
        """
        return {"vx": 0.0, "height": self.height, "settle": 1.0e9}

    def _wait_warmup(self, timeout: float = 120.0) -> None:
        """
        Waits until the warm-up of the simulator ends.
        """
        t0 = time.monotonic()
        while self.node.configure({"query": "info"})["warming"]:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError("warm-up did not end")
            time.sleep(0.02)

    def _start_controller(self) -> None:
        """
        Starts the controller and waits for its first commands.
        """
        node = self.node
        if self.ctl_proc is not None:
            self.ctl_proc.stop()

        # process
        cmd = [expand_environment(c) for c in self.ctl["launch"]["cmd"]]
        self.ctl_proc = Process(cmd, self.out / "logs" / f"{self.tag}_{self.ctl['name']}.log", self.env)

        # the robot stands while the controller starts
        node.configure({"reset": True, "defaults": True})
        self._wait_warmup()
        node.start(profile=self._stand(), duration=1.0e12)

        # first commands
        t0 = time.monotonic()
        while len(node.cmd_rx) < 50:
            if not self.ctl_proc.is_alive():
                raise RuntimeError(f"controller exited, see {self.ctl_proc.log}")
            if time.monotonic() - t0 > float(self.ctl.get("startup_timeout", 90.0)):
                raise TimeoutError(f"controller published no commands, see {self.ctl_proc.log}")
            time.sleep(0.05)
        node.stop()

    def _calibrate(self, candidates: list[float], probe: float = 3.0) -> float:
        """
        Returns the highest real-time factor at which both sides keep up.
        """
        node = self.node
        period = 1.0 / float(self.ctl["rate_hz"]) if self.ctl.get("rate_hz") else None
        best = 1.0
        self.calibration: list[dict[str, Any]] = []

        for k in candidates:

            # the robot stands for 'probe' s of sim time at the factor
            node.configure({"reset": True, "defaults": True, "rtf": k})
            self._wait_warmup()
            node.configure({"timing": True})
            node.start(profile=self._stand(), duration=1.0e12)

            start = node.clock
            t1 = time.monotonic()
            while node.clock - start < probe and time.monotonic() - t1 < probe / k * 3.0 + 10.0:
                time.sleep(0.02)
            node.stop()

            # simulator: speed and late frames
            timing = node.configure({"timing": True}).get("timing", {})
            frames = max(float(timing.get("frames", 0)), 1.0)
            overrun_frac = timing.get("overruns", 0) / frames

            # controller: share of missed cycles, and rate against the expected one
            cmd_rx = np.array(node.cmd_rx, dtype=float).reshape(-1, 2)
            gaps = np.diff(cmd_rx[:, 0]) if len(cmd_rx) > 10 else np.array([1.0])
            expected = period or float(np.median(gaps))
            missed = float(np.mean(gaps > 1.5 * expected + 0.002))
            ratio = (1.0 / float(np.median(gaps))) * expected if len(cmd_rx) > 10 else 0.0

            ok = (
                timing.get("rtf", 0.0) >= 0.97 * k
                and overrun_frac < metrics.DEFAULTS["overrun_frac"]
                and missed < 0.01
                and ratio >= 0.9
            )
            self.calibration.append({"rtf": k, "ok": bool(ok), "sim_rtf": timing.get("rtf"), "overrun_frac": overrun_frac, "cmd_missed_frac": missed, "cmd_rate_ratio": ratio})

            if not ok:
                break
            best = k

        node.configure({"rtf": best})

        return best
