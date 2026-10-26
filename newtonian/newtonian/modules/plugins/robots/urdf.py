
from __future__ import annotations

from typing import Any

import newton
import warp as wp

from newtonian.modules.plugins.robots.base import ROBOTS, Robot


@ROBOTS.register("urdf")
class RobotURDF(Robot):
    """
    Robot loaded from a URDF file.
    """

    def load(self, builder: newton.ModelBuilder, path: str, xform: wp.transform, floating: bool, args: dict[str, Any]) -> None:
        """
        Loads the URDF file into 'builder'.
        """
        builder.add_urdf(source=path, xform=xform, floating=floating, **args)
