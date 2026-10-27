
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import math

from geometry_msgs.msg import Twist

from newtonian.modules.arrays import to_numpy
from newtonian.modules.params import Parameters, ParameterError
from newtonian.modules.plugins.inputs.base import INPUTS, Input

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


_STEERING = ("left", "right")
_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass
class _Command:
    """
    Speed in m/s and steering angle in rad.
    """
    v: float = 0.0
    delta: float = 0.0


class _Base(Input):
    """
    Ackermann steering from 'Twist' messages.
    """
    msg_type = Twist
    default_timeout = 0.5

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the geometry, the limits, and the joint names.
        """
        super().__init__(name=name, p=p)

        self.wheelbase = p.get_float(key="wheelbase", default=0.33, above=0.0)
        self.track = p.get_float(key="track", default=0.24, above=0.0)
        self.wheel_radius = p.get_float(key="wheel_radius", default=0.05, above=0.0)
        self.max_steer = p.get_float(key="max_steer", default=0.4189, above=0.0)
        self.max_speed = p.get_float(key="max_speed", default=10.0, above=0.0)
        self.max_accel = p.get_float(key="max_accel", default=3.0, above=0.0)
        self.max_decel = p.get_float(key="max_decel", default=6.0, above=0.0)
        self.max_steer_rate = p.get_float(key="max_steer_rate", default=3.0, above=0.0)

        self.joints_steering: dict[str, str] = p.subtree(key="steer_joints").as_kwargs()
        self.joints_wheels: dict[str, str] = p.subtree(key="wheel_joints").as_kwargs()

        _check_keys(keys=self.joints_steering, allowed=_STEERING, where=f"{p.prefix}steer_joints")
        _check_keys(keys=self.joints_wheels, allowed=_WHEELS, where=f"{p.prefix}wheel_joints")

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Finds the indices of the steering and wheel joints.
        """
        super().on_model(sim=sim)

        joint_q_start = to_numpy(arr=sim.model.joint_q_start)
        joint_qd_start = to_numpy(arr=sim.model.joint_qd_start)
        self.steer = {side: int(joint_q_start[self.robot.find_joints(pattern=n)[0]]) for side, n in self.joints_steering.items()}
        self.wheels = {role: int(joint_qd_start[self.robot.find_joints(pattern=n)[0]]) for role, n in self.joints_wheels.items()}

    def _command(self, msg: Twist) -> _Command:
        """
        Returns the speed and steering angle of 'msg', within the limits.
        """
        v = msg.linear.x
        delta = 0.0 if abs(v) < 1.0e-3 else math.atan(self.wheelbase * msg.angular.z / v)
        return _Command(
            v=max(-self.max_speed, min(self.max_speed, v)),
            delta=max(-self.max_steer, min(self.max_steer, delta)),
        )

    def _write(self, sim: NewtonianSimulator, command: _Command) -> None:
        """
        Writes the steering angles and wheel speeds of 'command' into the simulator buffers.
        """
        v, delta = command.v, command.delta
        L, half = self.wheelbase, self.track / 2
        if abs(delta) < 1.0e-4:
            angles = {side: delta for side in _STEERING}
            speeds = {role: v for role in _WHEELS}
        else:
            R = L / math.tan(delta)
            w = v / R
            angles = {
                "left": math.atan(L / (R - half)),
                "right": math.atan(L / (R + half)),
            }
            speeds = {
                "front_left": math.copysign(1.0, R) * w * math.hypot(L, R - half),
                "front_right": math.copysign(1.0, R) * w * math.hypot(L, R + half),
                "rear_left": w * (R - half),
                "rear_right": w * (R + half),
            }
        for side, q in self.steer.items():
            sim.buffers.joint_target_q[q] = angles[side]
        for role, qd in self.wheels.items():
            sim.buffers.joint_target_qd[qd] = speeds[role] / self.wheel_radius
        sim.buffers.dirty = True


@INPUTS.register("racecar")
class InputRacecar(_Base):
    """
    Racecar input that ramps speed and steering to the last command.
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Starts at rest.
        """
        super().__init__(name=name, p=p)

        self.cmd_target = _Command()
        self.cmd = _Command()

    def control(self, sim: NewtonianSimulator) -> None:
        """
        Moves the command toward the target at the acceleration and steering rate limits.
        """
        super().control(sim=sim)

        if self.cmd_target == self.cmd:
            return
        dt = sim.frame_dt
        rate = self.max_accel if abs(self.cmd_target.v) > abs(self.cmd.v) and self.cmd_target.v * self.cmd.v >= 0.0 else self.max_decel
        self.cmd.v += max(-rate * dt, min(rate * dt, self.cmd_target.v - self.cmd.v))
        self.cmd.delta += max(-self.max_steer_rate * dt, min(self.max_steer_rate * dt, self.cmd_target.delta - self.cmd.delta))
        self._write(sim=sim, command=self.cmd)

    def handle(self, sim: NewtonianSimulator, msg: Twist) -> None:
        """
        Sets the target command.
        """
        self.cmd_target = self._command(msg=msg)

    def on_timeout(self, sim: NewtonianSimulator) -> None:
        """
        Sets the target to rest.
        """
        super().on_timeout(sim=sim)

        self.cmd_target = _Command()

    def reset(self) -> None:
        """
        Returns to rest.
        """
        super().reset()

        self.cmd_target = _Command()
        self.cmd = _Command()


def _check_keys(keys: dict[str, str], allowed: tuple[str, ...], where: str) -> None:
    """
    Raises 'ParameterError' if a key of 'keys' is not in 'allowed'.
    """
    unknown = sorted(set(keys) - set(allowed))
    if unknown:
        raise ParameterError(f"{where} expected keys in {list(allowed)} but got {unknown}")
