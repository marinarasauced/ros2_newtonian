
from __future__ import annotations

from typing import TYPE_CHECKING, Any, cast

import newton
import numpy as np
import warp as wp

from builtin_interfaces.msg import Time

from newtonian.modules.arrays import to_numpy
from newtonian.modules.params import ParameterError, Parameters
from newtonian.modules.registry import Registry
from newtonian.modules.transforms import TRANSFORM_ROS_TO_GL, convert_pose_to_transform, tf_mul

from newtonian.modules.plugins.base import Plugin
from newtonian.modules.plugins.robots.base import shorthand

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator
    from newtonian.modules.plugins.robots import Robot


SENSORS: Registry = Registry("sensor")



# --- Sensor Plugins -------------------------------------------------------------


class Sensor(Plugin):
    """
    Base of the sensor plugins, which publish one message type at a fixed rate.
    """
    msg_type: type
    default_qos = "sensor_data"

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the common sensor keys.
        """
        super().__init__(name=name, p=p)

        self.topic: str = p.get(key="topic", default=name)
        self.rate = p.get_float(key="rate", default=0.0, at_least=0.0)

        self.qos: str = p.get_choice(key="qos", choices=("default", "sensor_data"), default=self.default_qos)
        self.qos_depth = p.get_int(key="qos_depth", default=10, at_least=1)
        self.robot_name: str | None = p.get(key="robot", default=None)
        self.frame_id: str = p.get(key="frame_id", default="")
        self._due_time = 0.0

    def is_due(self, sim_time: float) -> bool:
        """
        True if the sensor must update.
        False otherwise.
        """
        if self.rate == 0.0:
            return True
        if sim_time + 1.0e-9 < self._due_time:
            return False
        period = 1.0 / self.rate
        self._due_time = max(self._due_time + period, sim_time + period * 0.5)
        return True

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Finds the robot that the 'robot' key refers to.
        """
        super().on_build(sim=sim, builder=builder)

        self.robot: Robot = sim.robots.resolve(name=self.robot_name, caller=f"sensor '{self.name}'")

    def reset(self) -> None:
        """
        Resets the due update schedule.
        """
        super().reset()

        self._due_time = 0.0

    def companions(self) -> list[Sensor]:
        """
        Returns extra sensors that publish next to this sensor.
        """
        return []

    def to_msg(self, stamp: Time) -> Any:
        """
        Returns the message of the last update.
        """
        raise NotImplementedError

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Reads the current state into the sensor data.
        """
        raise NotImplementedError


class SensorFixed(Sensor):
    """
    Sensor mounted on a robot at a site or at a body with an offset.
    """

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Finds the site, or adds a site at 'body' with 'offset'.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        # site assignment if configured
        if "site" in p:
            self.site_label = p.get(key="site")
            self.frame_id = self.frame_id or self.site_label
            return

        # site assignment if not configured
        body_name = p.get(key="body", default=None)
        prefix = self.robot.name + "/"
        bodies = [i for i, label in enumerate(builder.body_label) if label.startswith(prefix) and (body_name is None or shorthand(label=label) == body_name)]
        if not bodies:
            raise ParameterError(f"{p.prefix}body {body_name!r} is not a body of robot '{self.robot.name}'")
        self.site_label = f"{self.name}_site"
        builder.add_site(body=bodies[0], xform=convert_pose_to_transform(pose=p.get(key="offset", default=None), where=f"{p.prefix}offset"), label=f"{prefix}{self.site_label}")
        self.frame_id = self.frame_id or self.name

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Finds the body and the offset of the site.
        """
        super().on_model(sim=sim)

        self.site = self.robot.find_shapes(pattern=self.site_label)[0]
        self.body = int(to_numpy(arr=sim.model.shape_body)[self.site])
        self.offset = to_numpy(arr=sim.model.shape_transform)[self.site].astype(np.float64)

    def pose(self, cache: Cache) -> np.ndarray:
        """
        Returns the world pose of the site.
        """
        if self.body < 0:
            return self.offset
        return tf_mul(a=cache.body_q()[self.body].astype(np.float64), b=self.offset)


class SensorRaytrace(SensorFixed):
    """
    Mounted sensor that casts rays into the scene.
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Creates the ray tracer.
        """
        super().on_model(sim=sim)

        cfg = newton.sensors.SensorCamera.RenderConfig()
        cfg.max_distance = self.max_distance()

        self.tracer = newton.sensors.SensorCamera(model=sim.model, default_render_config=cfg)

    def max_distance(self) -> float:
        """
        Returns the maximum ray distance.
        """
        return self.p.get_float(key="max_distance", default=1000.0, above=0.0)

    def transform(self, sim: NewtonianSimulator, cache: Cache) -> wp.array:
        """
        Returns the pose of the site as a camera transform for the ray tracer.
        """
        X = tf_mul(a=self.pose(cache=cache), b=TRANSFORM_ROS_TO_GL).astype(np.float32)
        out = wp.array(data=[wp.transformf(wp.vec3f(*X[:3]), wp.quatf(*X[3:]))], dtype=wp.transformf, device=sim.device)
        return cast(typ="wp.array[wp.transformf]", val=out)



# --- Cache ----------------------------------------------------------------------


class Cache:
    """
    Cached data that several sensors may read from for a given frame.
    A new cache is computed for each update call, ensuring that it is never stale.
    """

    def __init__(self, sim: NewtonianSimulator) -> None:
        """
        Stores the simulator; the data is read on first use.
        """
        self.sim = sim
        self._body_q: np.ndarray | None = None
        self._bvh_shapes_refit = False

    def body_q(self) -> np.ndarray:
        """
        Returns the host copy of the body poses.
        """
        if self._body_q is None:
            self._body_q = to_numpy(arr=self.sim.state.body_q)
        return self._body_q

    def bvh_refit_shapes(self) -> None:
        """
        Refits the shape BVH for ray casts once per frame.
        """
        if not self._bvh_shapes_refit:
            self.sim.model.bvh_refit_shapes(state=self.sim.state)
            self._bvh_shapes_refit = True



# --- Helper Functions -----------------------------------------------------------


def noise(sim: NewtonianSimulator, std: float, shape: int | tuple[int, ...]) -> np.ndarray:
    """
    Returns Gaussian noise with standard deviation 'std', or zeros if 'std' is 0.
    """
    return sim.rng.normal(0.0, std, shape) if std > 0.0 else np.zeros(shape)


def update_sensors(sim: NewtonianSimulator, sensors: list[Sensor]) -> list[Sensor]:
    """
    Updates the sensors that are due, and returns them.
    """
    due = [s for s in sensors if s.is_due(sim_time=sim.sim_time)]
    cache = Cache(sim=sim)
    with wp.ScopedDevice(device=sim.device):
        for s in due:
            s.update(sim=sim, cache=cache)
    return due
