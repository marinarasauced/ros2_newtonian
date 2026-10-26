
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

from newtonian.modules.plugins.environments.base import ENVIRONMENTS, Environment

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@ENVIRONMENTS.register("groundplane")
class EnvironmentGroundPlane(Environment):
    """
    Infinite ground plane at 'height'.
    """
    default_color = (0.8, 0.8, 0.8)

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Adds the ground plane.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        builder.add_ground_plane(height=p.get_float(key="height", default=0.0), cfg=self.cfg, color=self.color)
