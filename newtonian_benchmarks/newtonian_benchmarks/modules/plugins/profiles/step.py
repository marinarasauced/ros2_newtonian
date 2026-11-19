
from __future__ import annotations

from newtonian_benchmarks.modules.plugins.profiles.base import PROFILES, Profile


@PROFILES.register("step")
class ProfileStep(Profile):
    """
    Full target command.
    """

    def factor(self, u: float) -> float:
        """
        Returns 1.
        """
        return 1.0
