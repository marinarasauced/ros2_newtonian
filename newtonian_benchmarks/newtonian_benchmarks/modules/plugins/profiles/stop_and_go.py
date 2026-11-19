
from __future__ import annotations

from newtonian_benchmarks.modules.plugins.profiles.base import PROFILES, Profile


@PROFILES.register("stop_and_go")
class ProfileStopGo(Profile):
    """
    Target command for a time, then zero.
    """

    def factor(self, u: float) -> float:
        """
        Returns 1 until the go time ends, then 0.
        """
        return 1.0 if u < float(self.p.get("go", 3.0)) else 0.0
