
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

from newtonian.modules.params import Parameters

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class Plugin:
    """
    Base of the environment, robot, input, and sensor plugins.
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Stores the entry name and its parameters.
        """
        self.name = name
        self.p = p

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before finalize.
        """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Called on the simulator after finalize.
        """

    def reset(self) -> None:
        """
        Called on each reset of the simulator.
        """
