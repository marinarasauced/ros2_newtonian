
from __future__ import annotations

from typing import Any

import newton
import warp as wp

from newtonian.modules.plugins.robots.base import ROBOTS, Robot


@ROBOTS.register("usd")
class RobotUSD(Robot):
    """
    Robot loaded from a USD file.
    """

    def load(self, builder: newton.ModelBuilder, path: str, xform: wp.transform, floating: bool, args: dict[str, Any]) -> None:
        """
        Loads the USD file into 'builder'.
        """
        builder.add_usd(source=path, xform=xform, floating=floating, **args)
