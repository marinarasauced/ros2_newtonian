
from __future__ import annotations

from typing import TYPE_CHECKING

import newton
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

from newtonian.modules.arrays import to_numpy
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(Sensor):
    """
    Net contact force on a group of robot bodies.
    """
    msg_type = WrenchStamped

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Creates the contact sensor for 'bodies', and asks the simulator for contacts.
        """
        super().on_model(sim=sim)
        p = self.p

        by_shapes = "shapes" in p
        find, labels = (self.robot.find_shapes, self.robot.shapes) if by_shapes else (self.robot.find_bodies, self.robot.bodies)
        patterns = p.get(key="shapes" if by_shapes else "bodies")
        items: list[int] = []
        for pattern in [patterns] if isinstance(patterns, str) else patterns:
            items += [i for i in find(pattern=pattern) if i not in items]
        self.names = [labels[i] for i in items]
        self.sensor = newton.sensors.SensorContact(model=sim.model, sensing_bodies=None if by_shapes else items, sensing_shapes=items if by_shapes else None)
        self.frame_id = self.frame_id or "world"
        sim.require_contacts()

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Reads the contact force on each body.
        """
        if sim.contacts is None:
            raise ValueError("sensor cannot update when contacts are missing")
        self.sensor.update(state=sim.state, contacts=sim.contacts)
        self.data = to_numpy(arr=self.sensor.total_force)

    def _wrench(self, stamp: Time, data: np.ndarray) -> WrenchStamped:
        """
        Returns 'data' as a wrench message.
        """
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.wrench.force.x = float(data[0])
        msg.wrench.force.y = float(data[1])
        msg.wrench.force.z = float(data[2])

        return msg


@SENSORS.register("contact_force")
class SensorContactForce(_Base):
    """
    Net contact force on all bodies in the world frame.
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        Returns the force vector.
        """
        return self._wrench(stamp=stamp, data=self.data.sum(axis=0))


@SENSORS.register("contact_scalar")
class SensorContactScalar(_Base):
    """
    Magnitude of the net contact force.
    """
    msg_type = JointState

    def to_msg(self, stamp: Time) -> JointState:
        """
        Returns the force magnitude.
        """
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.name = self.names
        msg.effort = np.linalg.norm(self.data, axis=1).astype(float).tolist()
        
        return msg
