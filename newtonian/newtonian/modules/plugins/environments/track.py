
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import newton
import numpy as np
import warp as wp

from PIL import Image
import yaml

from newtonian.modules.paths import resolve
from newtonian.modules.plugins.environments.base import ENVIRONMENTS, Environment

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@ENVIRONMENTS.register("track")
class EnvironmentTrack(Environment):
    """
    Walls built from a ROS map image.
    """

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Adds one box wall for each block of occupied pixels.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        mapping = Path(resolve(uri=p.get(key="map")))
        meta = yaml.safe_load(stream=mapping.read_text())
        image = np.asarray(Image.open(mapping.parent / meta["image"]).convert("L"), dtype=np.float32) / 255.0
        occ = image if meta.get("negate", 0) else 1.0 - image
        occupied = occ > float(meta.get("occupied_thresh", 0.65))

        res = float(meta["resolution"])
        ox, oy, yaw = (float(value) for value in meta["origin"][:3])
        cos, sin = np.cos(yaw), np.sin(yaw)
        height = p.get_float(key="wall_height", default=0.3, above=0.0)

        rectangles = []
        rows, cols = occupied.shape
        for row in range(rows):
            col = 0
            while col < cols:
                if not occupied[row, col]:
                    col += 1
                    continue
                col_end = col
                while col_end < cols and occupied[row, col_end]:
                    col_end += 1
                row_end = row + 1
                while row_end < rows and occupied[row_end, col:col_end].all():
                    row_end += 1
                occupied[row:row_end, col:col_end] = False
                rectangles.append((row, col, row_end, col_end))
                col = col_end

        for r0, c0, r1, c1 in rectangles:
            mx = (c0 + c1) / 2.0 * res
            my = (rows - (r0 + r1) / 2.0) * res
            x = ox + cos * mx - sin * my
            y = oy + sin * mx + cos * my
            builder.add_shape_box(body=-1, xform=wp.transform(wp.vec3(x, y, height / 2), wp.quat_rpy(0.0, 0.0, yaw)), hx=(c1-c0)*res/2, hy=(r1-r0)*res/2, hz=height/2, cfg=self.cfg, color=self.color)
