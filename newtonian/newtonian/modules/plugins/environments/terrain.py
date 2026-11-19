
from __future__ import annotations

from typing import TYPE_CHECKING

import math
import newton
import numpy as np
import warp as wp

from newtonian.modules.params import ParameterError
from newtonian.modules.plugins.environments.base import ENVIRONMENTS, Environment

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@ENVIRONMENTS.register("terrain")
class EnvironmentTerrain(Environment):
    """
    Heightfield terrain around the origin, with a flat disk of radius 'clear' at the start.
    """
    default_color = (0.8, 0.8, 0.8)

    def heights(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """
        Returns the heights and shape of the heightfield.
        """
        p = self.p
        kind = p.get_choice(key="kind", choices=("flat", "noise", "slope", "stairs", "step"), default="flat")
        X, Y = np.meshgrid(xs, ys)
        h = np.zeros_like(X)
        start = p.get_float(key="start", default=1.0)

        if kind == "noise":
            amplitude = p.get_float(key="amplitude", default=0.03, at_least=0.0)
            cell = p.get_float(key="cell", default=0.1, above=0.0)
            rng = np.random.default_rng(p.get_int(key="seed", default=0))
            nx, ny = int(math.ceil((xs[-1] - xs[0]) / cell)) + 1, int(math.ceil((ys[-1] - ys[0]) / cell)) + 1
            blocks = rng.uniform(0.0, amplitude, size=(ny, nx))
            ix = np.clip(((X - xs[0]) / cell).astype(int), 0, nx - 1)
            iy = np.clip(((Y - ys[0]) / cell).astype(int), 0, ny - 1)
            h = blocks[iy, ix]
        elif kind == "slope":
            angle = math.radians(p.get_float(key="angle", default=10.0, at_least=0.0))
            length = p.get_float(key="length", default=4.0, above=0.0)
            h = np.clip(X - start, 0.0, length) * math.tan(angle)
        elif kind == "stairs":
            height = p.get_float(key="step_height", default=0.08, above=0.0)
            length = p.get_float(key="step_length", default=0.35, above=0.0)
            count = p.get_int(key="steps", default=6, at_least=1)
            h = np.clip(np.floor((X - start) / length) + 1.0, 0.0, count) * height
        elif kind == "step":
            h = np.where(X >= start, p.get_float(key="step_height", default=0.08, above=0.0), 0.0)

        clear = p.get_float(key="clear", default=0.6, at_least=0.0)
        h = np.where(np.hypot(X, Y) < clear, 0.0, h)
        return h

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Adds the heightfield.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        # builder.add_ground_plane(height=p.get_float(key="height", default=0.0), cfg=self.cfg, color=self.color)
        hx = p.get_float(key="size_x", default=20.0, above=0.0)
        hy = p.get_float(key="size_y", default=10.0, above=0.0)
        resolution = p.get_float(key="resolution", default=0.05, above=0.0)
        
        cols = int(round(2 * hx / resolution)) + 1
        rows = int(round(2 * hy / resolution)) + 1
        if cols * rows > 8_000_000:
            raise ParameterError(f"{p.prefix}resolution gives {cols * rows} cells which exceeds allowed quantity")

        h = self.heights(xs=np.linspace(-hx, hx, cols), ys=np.linspace(-hy, hy, rows)).astype(np.float32)
        self.hmax = float(h.max())
        field = newton.Heightfield(data=h, nrow=rows, ncol=cols, hx=hx, hy=hy, min_z=0.0, max_z=max(self.hmax, 1.0e-3))

        builder.add_shape_heightfield(xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()), heightfield=field, cfg=self.cfg, color=self.color)
