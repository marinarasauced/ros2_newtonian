
from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np
import warp as wp

from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan

from newtonian.modules.params import Parameters
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, SensorRaytrace, noise

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(SensorRaytrace):
    """
    2D lidar that scans in the xy plane of its site.
    """
    msg_type = LaserScan

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the scan keys.
        """
        super().__init__(name=name, p=p)

        self.samples = p.get_int(key="samples", default=1080, at_least=1)
        self.angle_min = p.get_float(key="angle_min", default=-2.356194)
        self.angle_max = p.get_float(key="angle_max", default=2.356194)
        self.range_min = p.get_float(key="range_min", default=0.06, at_least=0.0)
        self.range_max = p.get_float(key="range_max", default=30.0, above=0.0)
        self.noise_std = p.get_float(key="noise_std", default=0.0, at_least=0.0)

    def max_distance(self) -> float:
        """
        Returns 'range_max'.
        """
        return self.range_max

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Computes the ray directions, and creates the range output.
        """
        super().on_model(sim=sim)

        n = self.samples
        self.angles = np.linspace(start=self.angle_min, stop=self.angle_max, num=n)

        rays = np.zeros(shape=(1, n, 1, 2, 3), dtype=np.float32)
        rays[0, :, 0, 1, 0] = -np.sin(self.angles)
        rays[0, :, 0, 1, 2] = -np.cos(self.angles)
        self.rays = cast(typ="wp.array4d[wp.vec3f]", val=wp.array(data=rays, dtype=wp.vec3f, device=sim.device))
        self.depth = self.tracer.allocate_image_depth(view_count=1, width=n, height=1)
        self.period = 1.0 / self.rate if self.rate > 0.0 else sim.frame_dt

    def _laserscan(self, stamp: Time, data: np.ndarray) -> LaserScan:
        """
        Returns 'data' as a laser scan message.
        """
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / max(self.samples - 1, 1)
        msg.time_increment = 0.0
        msg.scan_time = self.period
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = data.astype(np.float32).tolist()

        return msg


@SENSORS.register("lidar")
class SensorLidar(_Base):
    """
    2D lidar with Gaussian noise; no hit is inf.
    """

    def to_msg(self, stamp: Time) -> LaserScan:
        """
        Returns the last scan.
        """
        return self._laserscan(stamp=stamp, data=self.data)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Casts the rays, and adds noise to the ranges.
        """
        cache.bvh_refit_shapes()
        self.tracer.update(state=sim.state, camera_transforms=self.transform(sim=sim, cache=cache), camera_rays=self.rays, depth_image=self.depth)
        ranges = cast(typ="wp.array", val=self.depth).numpy().reshape(-1).astype(np.float64)
        misses = (ranges <= 0.0) | (ranges > self.range_max)
        ranges = ranges + noise(sim=sim, std=self.noise_std, shape=ranges.shape)
        ranges[ranges < self.range_min] = -np.inf
        ranges[misses] = np.inf
        self.data = ranges
