
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from newtonian.modules.registry import Registry
from newtonian_benchmarks.modules.plugins.base import Plugin
from newtonian_benchmarks.modules.plugins.profiles import Command


METRICS: Registry = Registry("metric")


DEFAULTS = {
    "fall_tilt": 1.0,            # rad, angle between body z and world z
    "fall_height": 0.10,         # m, base height
    "rp_limit": 0.35,            # rad, roll and pitch limit for the exceedance count
    "transient": 1.0,            # s after the start of the profile that the steady-state RMSE excludes
    "recover_tilt": 0.10,        # rad above the tilt before the push
    "recover_vel": 0.15,         # m/s above the velocity error before the push (heading frame)
    "recover_wz": 0.30,          # rad/s above the yaw-rate error before the push
    "recover_smooth": 0.3,       # s, window of the moving average of the errors
    "recover_hold": 0.5,         # s that the recovery condition must hold
    "recover_window": 4.0,       # s after the end of a push
    "overrun_frac": 0.10,        # share of frames that took longer than the frame period (the timer catches up, so single late frames are not a failure)
    "gap_limit": 0.1,            # s, longest allowed pause of the controller commands
}


def euler(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns the roll, pitch, and yaw of the quaternions 'q'.
    """
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Returns the vectors 'v' rotated by the quaternions 'q'.
    """
    u = q[:, :3]
    w = q[:, 3:4]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def root_mean_square(x: np.ndarray) -> float:
    """
    Returns the root mean square of 'x'.
    """
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def count_episodes(mask: np.ndarray) -> int:
    """
    Returns the number of runs of True in 'mask'.
    """
    return int(np.sum(mask[1:] & ~mask[:-1]) + (1 if len(mask) and mask[0] else 0))


@dataclass
class Context:
    """
    Recorded trial and the signals that the metrics share.
    """
    th: dict[str, Any]
    settle: float
    pushes: list[dict[str, Any]]
    js: list
    cmd_rx: np.ndarray
    timing: dict[str, Any]
    rtf_target: float
    t: np.ndarray
    pos: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    tilt: np.ndarray
    v_h: np.ndarray            # velocity in the heading frame (x, y)
    wz: np.ndarray             # yaw rate, world
    cmd: np.ndarray            # command at each sample: vx, vy, wz, height
    fell: bool
    t_fall: float
    t_end: float               # end of the run: the fall, or the last sample
    active: np.ndarray         # samples after the settle time, up to t_end
    steady: np.ndarray         # active samples after the transient

    @classmethod
    def from_trial(cls, odom: np.ndarray, js: list, cmd_rx: np.ndarray, profile: dict[str, Any], pushes: list[dict[str, Any]], timing: dict[str, Any], rtf_target: float, th: dict[str, Any]) -> Context:
        """
        Returns the context of a trial.
        """
        settle = float(profile.get("settle", 1.0))
        # signals of the odometry
        t = odom[:, 0]
        pos, q = odom[:, 1:4], odom[:, 4:8]
        roll, pitch, yaw = euler(q=q)
        tilt = np.arccos(np.clip(1.0 - 2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1.0, 1.0))
        v_world = rotate(q=q, v=odom[:, 8:11])
        c, s = np.cos(yaw), np.sin(yaw)
        v_h = np.stack([c * v_world[:, 0] + s * v_world[:, 1], -s * v_world[:, 0] + c * v_world[:, 1]], axis=1)
        wz = rotate(q=q, v=odom[:, 11:14])[:, 2]

        # fall
        fall = (tilt > th["fall_tilt"]) | (pos[:, 2] < th["fall_height"])
        fell = bool(fall.any())
        t_fall = float(t[np.argmax(fall)]) if fell else float("nan")
        t_end = t_fall if fell else float(t[-1])

        # command at each sample, and the samples that count
        command = Command(p=profile)
        cmd = np.array([command.at(t=float(ti)) for ti in t])
        active = (t >= settle) & (t <= t_end)
        steady = active & (t >= settle + th["transient"])
        return cls(th=th, settle=settle, pushes=pushes, js=js, cmd_rx=cmd_rx, timing=timing, rtf_target=rtf_target, t=t, pos=pos, roll=roll, pitch=pitch, tilt=tilt, v_h=v_h, wz=wz, cmd=cmd, fell=fell, t_fall=t_fall, t_end=t_end, active=active, steady=steady)


class Metric(Plugin):
    """
    Base of the metrics.
    """

    def evaluate(self, ctx: Context, out: dict[str, Any]) -> None:
        """
        Adds the values of this metric to 'out'.
        """
        raise NotImplementedError
