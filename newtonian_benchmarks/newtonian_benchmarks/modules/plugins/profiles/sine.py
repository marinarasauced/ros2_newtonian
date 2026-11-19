
from __future__ import annotations

import math

from newtonian_benchmarks.modules.plugins.profiles.base import PROFILES, Profile


@PROFILES.register("sine")
class ProfileSine(Profile):
    """
    Target command that follows a sine.
    """

    def factor(self, u: float) -> float:
        """
        Returns the sine at time 'u'.
        """
        return math.sin(2.0 * math.pi * float(self.p.get("freq", 0.25)) * u)
