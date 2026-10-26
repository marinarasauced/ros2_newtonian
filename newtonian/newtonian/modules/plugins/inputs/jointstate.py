
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import math

from sensor_msgs.msg import JointState

from newtonian.modules.arrays import to_numpy
from newtonian.modules.params import Parameters
from newtonian.modules.plugins.inputs.base import INPUTS, Input
from newtonian.modules.plugins.robots.base import JOINT_TYPES

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@dataclass
class _Command:
    """
    Joint targets by name; 'None' keeps the current value.
    """
    names: list[str]
    position: list[float | None]
    velocity: list[float | None]
    effort: list[float | None]


class _Base(Input):
    """
    Joint targets from 'JointState' messages.
    """
    msg_type = JointState

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Maps the names of the revolute and prismatic joints to their indices.
        """
        super().on_model(sim=sim)

        joint_type = to_numpy(arr=sim.model.joint_type)
        joint_q_start = to_numpy(arr=sim.model.joint_q_start)
        joint_qd_start = to_numpy(arr=sim.model.joint_qd_start)
        self.index: dict[str, tuple[int, int]] = {name: (int(joint_q_start[j]), int(joint_qd_start[j])) for j, name in self.robot.joints.items() if joint_type[j] in JOINT_TYPES}

    def _command(self, msg: JointState) -> _Command:
        """
        Returns the values of 'msg', with missing and NaN values as 'None'.
        """
        def values(items: Sequence[float]) -> list[float | None]:
            return [float(items[i]) if i < len(items) and math.isfinite(items[i]) else None for i in range(len(msg.name))]
        return _Command(
            names=list(msg.name),
            position=values(items=msg.position),
            velocity=values(items=msg.velocity),
            effort=values(items=msg.effort),
        )

    def _write(self, sim: NewtonianSimulator, command: _Command) -> None:
        """
        Writes the command into the simulator buffers; unknown names are ignored.
        """
        for name, position, velocity, effort in zip(command.names, command.position, command.velocity, command.effort):
            idx = self.index.get(name)
            if idx is None:
                continue
            q_idx, qd_idx = idx
            if position is not None:
                sim.buffers.joint_target_q[q_idx] = position
            if velocity is not None:
                sim.buffers.joint_target_qd[qd_idx] = velocity
            if effort is not None:
                sim.buffers.joint_f[qd_idx] = effort
        sim.buffers.dirty = True


@INPUTS.register("jointstate")
class InputJointState(_Base):
    """
    Joint position, velocity, and effort targets for any robot.
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Starts with no message handled.
        """
        super().__init__(name=name, p=p)

        self._handled: JointState | None = None

    def handle(self, sim: NewtonianSimulator, msg: JointState) -> None:
        """
        Writes each message once.
        """
        if msg is self._handled:
            return
        self._write(sim=sim, command=self._command(msg=msg))
        self._handled = msg

    def reset(self) -> None:
        """
        Clears the handled message.
        """
        super().reset()

        self._handled = None
