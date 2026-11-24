
from __future__ import annotations

from newtonian_benchmarks.modules.plugins.profiles.base import PROFILES, Profile


@PROFILES.register("ramp")
class ProfileRamp(Profile):
    """
    Target command that rises linearly, then holds.
    """

    def factor(self, u: float) -> float:
        """
        Returns the share of the ramp that has passed.
        """
        return min(u / max(float(self.p.get("ramp", 2.0)), 1.0e-6), 1.0)
