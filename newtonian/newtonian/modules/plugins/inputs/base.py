
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import newton

from newtonian.modules.params import Parameters
from newtonian.modules.registry import Registry

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator
    from newtonian.modules.plugins.robots import Robot


INPUTS: Registry = Registry("input")



# --- Input Plugins --------------------------------------------------------------


class Input(Plugin):
    """
    Base of the input plugins, which turn messages from one topic into robot commands.
    """
    msg_type: type
    default_qos = "default"
    default_timeout = 0.0

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the common input keys.
        """
        super().__init__(name=name, p=p)

        self.topic: str = p.get(key="topic", default=name)

        self.qos: str = p.get_choice(key="qos", choices=("default", "sensor_data"), default=self.default_qos)
        self.qos_depth = p.get_int(key="qos_depth", default=10, at_least=1)
        self.robot_name: str | None = p.get(key="robot", default=None)
        self.timeout = p.get_float(key="timeout", default=self.default_timeout, at_least=0.0)
        self._msg = None
        self._msg_time = None

    def callback(self, msg: Any) -> None:
        """
        Stores the last message; the timeout starts at the next step.
        """
        self._msg = msg
        self._msg_time = None

    def control(self, sim: NewtonianSimulator) -> None:
        """
        Applies the last message, or calls 'on_timeout' if it is older than 'timeout'.
        """
        msg = self._msg
        if msg is None:
            return
        if self._msg_time is None:
            self._msg_time = sim.sim_time
        if self.timeout > 0.0 and sim.sim_time - self._msg_time > self.timeout:
            self._msg = None
            self.on_timeout(sim=sim)
            return
        self.handle(sim=sim, msg=msg)
        self.robot.activate()

    def handle(self, sim: NewtonianSimulator, msg: Any) -> None:
        """
        Writes the command of 'msg' into the simulator buffers.
        """
        raise NotImplementedError

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Finds the robot that the 'robot' key refers to.
        """
        super().on_build(sim=sim, builder=builder)

        self.robot: Robot = sim.robots.resolve(name=self.robot_name, caller=f"input '{self.name}'")

    def on_timeout(self, sim: NewtonianSimulator) -> None:
        """
        Called once when the last message is older than 'timeout'.
        """

    def reset(self) -> None:
        """
        Clears the last message.
        """
        super().reset()

        self._msg = None
        self._msg_time = None
