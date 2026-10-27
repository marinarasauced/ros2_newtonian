
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState

from newtonian.modules.arrays import to_numpy
from newtonian.modules.plugins.robots.base import JOINT_TYPES
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@dataclass
class _Data:
    """
    Joint positions, velocities, and efforts.
    """
    q: np.ndarray
    qd: np.ndarray
    effort: np.ndarray | None


class _Base(Sensor):
    """
    State of the revolute and prismatic joints of a robot.
    """
    msg_type = JointState
    default_qos = "default"

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Finds the joints, and asks for actuator forces if the solver gives them.
        """
        super().on_model(sim=sim)
        p = self.p

        joint_type = to_numpy(arr=sim.model.joint_type)
        joint_q_start = to_numpy(arr=sim.model.joint_q_start)
        joint_qd_start = to_numpy(arr=sim.model.joint_qd_start)
        patterns = p.get(key="joints", default="*")
        joints: list[int] = []
        for pattern in [patterns] if isinstance(patterns, str) else patterns:
            joints += [j for j in self.robot.find_joints(pattern=pattern) if joint_type[j] in JOINT_TYPES and j not in joints]
        self.names = [self.robot.joints[j] for j in joints]
        self.joint_q_idx = joint_q_start[joints]
        self.joint_qd_idx = joint_qd_start[joints]

        self.publish_effort = sim.solver_plugin.reports_actuator_forces
        if self.publish_effort:
            sim.model.request_state_attributes("mujoco:qfrc_actuator")

    def _jointstate(self, stamp: Time, data: _Data) -> JointState:
        """
        Returns 'data' as a joint state message.
        """
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.name = self.names
        msg.position = data.q.astype(float).tolist()
        msg.velocity = data.qd.astype(float).tolist()
        if data.effort is not None:
            msg.effort = data.effort.astype(float).tolist()

        return msg


@SENSORS.register("jointstate")
class SensorJointState(_Base):
    """
    Joint state; effort only if the solver gives actuator forces.
    """

    def to_msg(self, stamp: Time) -> JointState:
        """
        Returns the last joint state message.
        """
        return self._jointstate(stamp=stamp, data=self.data)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Reads the joint positions, velocities, and efforts.
        """
        self.data = _Data(
            q=to_numpy(arr=sim.state.joint_q)[self.joint_q_idx],
            qd=to_numpy(arr=sim.state.joint_qd)[self.joint_qd_idx],
            effort=to_numpy(arr=getattr(sim.state, "mujoco").qfrc_actuator)[self.joint_qd_idx] if self.publish_effort else None,
        )
