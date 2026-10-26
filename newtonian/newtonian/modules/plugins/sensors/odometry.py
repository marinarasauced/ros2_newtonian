
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from newtonian.modules.arrays import to_numpy
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor, noise
from newtonian.modules.transforms import quat_rotate, quat_rotate_inv

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@dataclass
class _Data:
    """
    Pose in the world frame, and velocities in the body frame.
    """
    pose: np.ndarray
    v: np.ndarray
    w: np.ndarray


class _Base(Sensor):
    """
    Odometry of one robot body.
    """
    msg_type = Odometry

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Finds the body, and reads the frame and noise keys.
        """
        super().on_model(sim=sim)
        p = self.p

        self.body = self.robot.find_body(name=p.get(key="body", default=None))
        self.frame_id = self.frame_id or "odom"
        self.child_frame_id = p.get(key="child_frame_id", default=self.robot.bodies[self.body])
        self.com = to_numpy(arr=sim.model.body_com)[self.body].astype(np.float64)
        self.pose_std = p.get_float(key="pose_noise_std", default=0.0, at_least=0.0)
        self.twist_std = p.get_float(key="twist_noise_std", default=0.0, at_least=0.0)

    def _odometry(self, stamp: Time, data: _Data) -> Odometry:
        """
        Returns 'data' as an odometry message, with covariances from the noise.
        """
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        p = data.pose[:3]
        msg.pose.pose.position.x = float(p[0])
        msg.pose.pose.position.y = float(p[1])
        msg.pose.pose.position.z = float(p[2])

        q = data.pose[3:]
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        v = data.v
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = float(v[2])

        w = data.w
        msg.twist.twist.angular.x = float(w[0])
        msg.twist.twist.angular.y = float(w[1])
        msg.twist.twist.angular.z = float(w[2])

        msg.pose.covariance = np.diag([self.pose_std**2] * 3 + [0.0] * 3).flatten().tolist()
        msg.twist.covariance = np.diag([self.twist_std**2] * 6).flatten().tolist()

        return msg


@SENSORS.register("odometry")
class SensorOdometry(_Base):
    """
    Odometry from the true body state with Gaussian noise.
    """

    def to_msg(self, stamp: Time) -> Odometry:
        """
        Returns the last odometry message.
        """
        return self._odometry(stamp=stamp, data=self.data)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Reads the pose and the velocities of the body origin, and adds noise.
        """
        X = cache.body_q()[self.body].astype(np.float64)
        q = X[3:]
        qd = to_numpy(arr=sim.state.body_qd)[self.body].astype(np.float64)
        v_com, w = qd[:3], qd[3:]
        v_origin = v_com + np.cross(a=w, b=-quat_rotate(quat=q, vec=self.com))
        pose = X.copy()
        pose[:3] += noise(sim=sim, std=self.pose_std, shape=3)
        self.data = _Data(
            pose=pose,
            v=quat_rotate_inv(quat=q, vec=v_origin+noise(sim=sim, std=self.twist_std, shape=3)),
            w=quat_rotate_inv(quat=q, vec=w)+noise(sim=sim, std=self.twist_std, shape=3),
        )
