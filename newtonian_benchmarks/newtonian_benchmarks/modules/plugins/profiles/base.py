
from __future__ import annotations

from typing import Any

from newtonian.modules.registry import Registry
from newtonian_benchmarks.modules.plugins.base import Plugin


PROFILES: Registry = Registry("profile")


class Profile(Plugin):
    """
    Base of the command profiles.
    """

    def factor(self, u: float) -> float:
        """
        Returns the scale of the target command at time 'u'.
        """
        raise NotImplementedError


class Command:
    """
    Command of one trial.
    """

    def __init__(self, p: dict[str, Any]) -> None:
        """
        Reads the command keys of 'p'.
        """
        self.height = float(p["height"])
        self.settle = float(p.get("settle", 1.0))
        self.target = (float(p.get("vx", 0.0)), float(p.get("vy", 0.0)), float(p.get("wz", 0.0)))
        self.profile: Profile = PROFILES.get(p.get("profile", "step"))(p.get("profile", "step"), p)

    def at(self, t: float) -> tuple[float, float, float, float]:
        """
        Returns the command at time 't'.
        """
        if t < self.settle:
            return 0.0, 0.0, 0.0, self.height
        k = self.profile.factor(t - self.settle)
        return self.target[0] * k, self.target[1] * k, self.target[2] * k, self.height
