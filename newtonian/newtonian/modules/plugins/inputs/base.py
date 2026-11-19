
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

        self.latency = p.get_float(key="latency", default=0.0, at_least=0.0)
        self.jitter = p.get_float(key="jitter", default=0.0, at_least=0.0)
        self.drop = p.get_float(key="drop", default=0.0, at_least=0.0)

        self._arrived: list[Any] = []
        self._queue: list[tuple[float, Any]] = []

    def callback(self, msg: Any) -> None:
        """
        Stores the last message; the timeout starts at the next step.
        """
        if self.latency == 0.0 and self.jitter == 0.0 and self.drop == 0.0 and not self._arrived and not self._queue:
            self._msg = msg
            self._msg_time = None
            return
        self._arrived.append(msg)

    def control(self, sim: NewtonianSimulator) -> None:
        """
        Applies the last message, or calls 'on_timeout' if it is older than 'timeout'.
        """
        self._release(sim=sim)
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

    def _release(self, sim: NewtonianSimulator) -> None:
        """
        Moves any messages that have arrived into the delay queue and updates the current due message.
        """
        arrived, self._arrived = self._arrived, []
        for msg in arrived:
            if self.drop > 0.0 and sim.rng.random() < self.drop:
                continue
            delay = self.latency + (float(sim.rng.uniform(low=-self.jitter, high=self.jitter)) if self.jitter > 0.0 else 0.0)
            self._queue.append((sim.sim_time + max(delay, 0.0), msg))

        if not self._queue:
            return

        self._queue.sort(key=lambda item: item[0])
        now = sim.sim_time + 1.0e-9
        due = [m for t, m in self._queue if t <= now]
        if due:
            self._queue = [(t, m) for t, m in self._queue if t > now]
            self._msg = due[-1]
            self._msg_time = None

    def reset(self) -> None:
        """
        Clears the last message.
        """
        super().reset()

        self._msg = None
        self._msg_time = None
        self._arrived = []
        self._queue = []
