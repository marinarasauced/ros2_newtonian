
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

from newtonian.modules.params import Parameters, ParameterError
from newtonian.modules.registry import Registry

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


ENVIRONMENTS: Registry = Registry("environment")



# --- Environment Plugins --------------------------------------------------------


class Environment(Plugin):
    """
    Base of the environment plugins, which add static shapes to the scene.
    """
    default_color = (0.6, 0.6, 0.6)

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the 'color' key.
        """
        super().__init__(name=name, p=p)

        color = p.get(key="color", default=self.default_color)
        if not isinstance(color, list) or len(color) != 3 or not all(isinstance(c, (int, float)) for c in color):
            raise ParameterError(f"{p.prefix}color expected [r, g, b] but got {color!r}")
        self.color: tuple[float, float, float] = (float(color[0]), float(color[1]), float(color[2]))

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Reads the contact keys into a shape config.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        cfg = builder.default_shape_cfg.copy()
        for key in ("mu", "restitution", "ke", "kd", "kf", "margin"):
            if key in p:
                setattr(cfg, key, p.get_float(key=key, at_least=0.0))
        self.cfg = cfg
