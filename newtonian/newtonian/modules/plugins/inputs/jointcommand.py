
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import math

import newton
import numpy as np

from newtonian_msgs.msg import JointCommand

from newtonian.modules.arrays import require, to_numpy
from newtonian.modules.params import Parameters
from newtonian.modules.plugins.inputs.base import INPUTS, Input
from newtonian.modules.plugins.robots.base import JOINT_TYPES

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@dataclass
class _Command:
    """
    Joint targets and gains by name; 'None' keeps the current value, except for the effort, which is 0.
    """
    names: list[str]
    position: list[float | None]
    velocity: list[float | None]
    effort: list[float | None]
    kp: list[float | None]
    kd: list[float | None]


class _Base(Input):
    """
    Joint targets and gains from 'JointCommand' messages.
    """
    msg_type = JointCommand

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Starts with no gains read from the model.
        """
        super().__init__(name=name, p=p)

        self._sim: NewtonianSimulator | None = None
        self._initial: dict[int, tuple[float, float]] = {}
        self._gains: dict[int, tuple[float, float]] | None = None

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Maps the names of the revolute and prismatic joints to their indices, and keeps their gains from the model.
        """
        super().on_model(sim=sim)

        joint_type = to_numpy(arr=sim.model.joint_type)
        joint_q_start = to_numpy(arr=sim.model.joint_q_start)
        joint_qd_start = to_numpy(arr=sim.model.joint_qd_start)
        self.index: dict[str, tuple[int, int]] = {name: (int(joint_q_start[j]), int(joint_qd_start[j])) for j, name in self.robot.joints.items() if joint_type[j] in JOINT_TYPES}
        self._sim = sim
        self._initial = self._read_gains(sim=sim)
        self._gains = None

    def _command(self, msg: JointCommand) -> _Command:
        """
        Returns the values of 'msg', with missing and NaN values as 'None', and negative gains as 'None'.
        """
        def values(items: Sequence[float], at_least: float | None = None) -> list[float | None]:
            return [float(items[i]) if i < len(items) and math.isfinite(items[i]) and (at_least is None or items[i] >= at_least) else None for i in range(len(msg.name))]
        return _Command(
            names=list(msg.name),
            position=values(items=msg.position),
            velocity=values(items=msg.velocity),
            effort=values(items=msg.effort),
            kp=values(items=msg.kp, at_least=0.0),
            kd=values(items=msg.kd, at_least=0.0),
        )

    def _write(self, sim: NewtonianSimulator, command: _Command) -> None:
        """
        Writes the targets into the simulator buffers and the gains that changed into the model; unknown names are ignored.
        """
        if self._gains is None:
            self._gains = self._read_gains(sim=sim)
        changes: dict[int, tuple[float, float]] = {}
        for name, position, velocity, effort, kp, kd in zip(command.names, command.position, command.velocity, command.effort, command.kp, command.kd):
            idx = self.index.get(name)
            if idx is None:
                continue
            q_idx, qd_idx = idx
            if position is not None:
                sim.buffers.joint_target_q[q_idx] = position
            if velocity is not None:
                sim.buffers.joint_target_qd[qd_idx] = velocity
            sim.buffers.joint_f[qd_idx] = effort if effort is not None else 0.0
            ke_now, kd_now = self._gains[qd_idx]
            gains = (ke_now if kp is None else float(np.float32(kp)), kd_now if kd is None else float(np.float32(kd)))
            if gains != (ke_now, kd_now):
                changes[qd_idx] = gains
        sim.buffers.dirty = True
        if changes:
            self._set_gains(sim=sim, changes=changes)

    def _read_gains(self, sim: NewtonianSimulator) -> dict[int, tuple[float, float]]:
        """
        Returns the gains 'ke' and 'kd' of the model for the joints of this input, by index.
        """
        ke = to_numpy(arr=sim.model.joint_target_ke)
        kd = to_numpy(arr=sim.model.joint_target_kd)
        return {qd_idx: (float(ke[qd_idx]), float(kd[qd_idx])) for _, qd_idx in self.index.values()}

    def _set_gains(self, sim: NewtonianSimulator, changes: dict[int, tuple[float, float]]) -> None:
        """
        Writes 'changes' (gains by index) to the model, and notifies the solver.
        """
        for col, arr in enumerate((require(arr=sim.model.joint_target_ke), require(arr=sim.model.joint_target_kd))):
            values = arr.numpy()
            for qd_idx, gains in changes.items():
                values[qd_idx] = gains[col]
            arr.assign(values)
        sim.solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)
        if self._gains is not None:
            self._gains.update(changes)

    def _restore_gains(self) -> None:
        """
        Returns the gains to those of the model at the start, if a message changed them.
        """
        if self._sim is not None and self._gains is not None:
            self._set_gains(sim=self._sim, changes=self._initial)
        self._gains = None


@INPUTS.register("jointcommand")
class InputJointCommand(_Base):
    """
    Joint position, velocity, and effort targets, with the gains of each joint, for any robot.
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Starts with no message handled.
        """
        super().__init__(name=name, p=p)

        self._handled: JointCommand | None = None

    def handle(self, sim: NewtonianSimulator, msg: JointCommand) -> None:
        """
        Writes each message once. The lazy drives turn on first, so the gains of the message replace those of the yaml.
        """
        if msg is self._handled:
            return
        self.robot.activate()
        self._write(sim=sim, command=self._command(msg=msg))
        self._handled = msg

    def reset(self) -> None:
        """
        Clears the handled message, and returns the gains to those of the model at the start.
        """
        super().reset()

        self._handled = None
        self._restore_gains()
