
from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np
import warp as wp

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image

from newtonian.modules.params import Parameters
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor, SensorRaytrace

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(SensorRaytrace):
    """
    Pinhole camera that publishes an image.
    """
    msg_type = Image

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the 'camera_info' key.
        """
        super().__init__(name=name, p=p)

        self.publish_info = p.get_bool(key="camera_info", default=True)

    def companions(self) -> list[Sensor]:
        """
        Returns the camera info sensor, next to the image topic.
        """
        if not self.publish_info:
            return []
        head, _, _ = self.topic.rpartition("/")
        topic = f"{head}/camera_info" if head else f"{self.topic}/camera_info"
        return [_CameraInfo(camera=self, topic=topic)]

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Reads the image size and field of view, and computes the camera rays.
        """
        super().on_model(sim=sim)
        p = self.p

        self.width = p.get_int(key="width", default=640, at_least=1)
        self.height = p.get_int(key="height", default=480, at_least=1)
        self.fov = p.get_float(key="fov", default=1.0, above=0.0)
        self.rays = self.tracer.compute_camera_rays_pinhole(width=self.width, height=self.height, camera_fov=self.fov, device=sim.device)

    def _image(self, stamp: Time, data: np.ndarray, encoding: str, step: int) -> Image:
        """
        Returns 'data' as an image message.
        """
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.height, msg.width = self.height, self.width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data.tobytes()

        return msg


@SENSORS.register("camera_color")
class SensorCameraColor(_Base):
    """
    Color camera, 'rgb8'.
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Creates the color image output.
        """
        super().on_model(sim=sim)

        self.color = self.tracer.allocate_image_color(view_count=1, width=self.width, height=self.height)

    def to_msg(self, stamp: Time) -> Image:
        """
        Returns the last color image.
        """
        return self._image(stamp=stamp, data=self.data, encoding="rgb8", step=self.width*3)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Renders the color image.
        """
        cache.bvh_refit_shapes()
        self.tracer.update(state=sim.state, camera_transforms=self.transform(sim=sim, cache=cache), camera_rays=self.rays, color_image=self.color)
        rgba = cast(typ="wp.array", val=self.color).numpy().reshape(self.height, self.width).view(np.uint8).reshape(self.height, self.width, 4)
        self.data = np.ascontiguousarray(rgba[..., :3])


@SENSORS.register("camera_depth")
class SensorCameraDepth(_Base):
    """
    Depth camera, '32FC1' in m; no hit is NaN.
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Creates the depth image output.
        """
        super().on_model(sim=sim)

        self.depth = self.tracer.allocate_image_depth_forward(view_count=1, width=self.width, height=self.height)

    def to_msg(self, stamp: Time) -> Image:
        """
        Returns the last depth image.
        """
        return self._image(stamp=stamp, data=self.data, encoding="32FC1", step=self.width*4)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Renders the depth image.
        """
        cache.bvh_refit_shapes()
        self.tracer.update(state=sim.state, camera_transforms=self.transform(sim=sim, cache=cache), camera_rays=self.rays, forward_depth_image=self.depth)
        depth = cast(typ="wp.array", val=self.depth).numpy().reshape(self.height, self.width).astype(np.float32)
        depth[depth <= 0.0] = np.nan
        self.data = depth


class _CameraInfo(Sensor):
    """
    Camera info of a camera, at the camera rate.
    """
    msg_type = CameraInfo

    def __init__(self, camera: _Base, topic: str) -> None:
        """
        Uses the topic, rate, QoS, and robot of 'camera'.
        """
        p = Parameters(flat={"topic": topic, "rate": camera.rate, "qos": camera.qos, "qos_depth": camera.qos_depth, "robot": camera.robot_name}, prefix=f"{camera.p.prefix}camera_info.")
        super().__init__(name=f"{camera.name}/camera_info", p=p)

        self.camera = camera

    def to_msg(self, stamp: Time) -> CameraInfo:
        """
        Returns the pinhole model of the camera.
        """
        camera = self.camera
        f = float((camera.height / 2) / np.tan(camera.fov / 2))
        cx, cy = (camera.width - 1) / 2, (camera.height - 1) / 2

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = camera.frame_id
        msg.height, msg.width = camera.height, camera.width
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Does nothing; the camera info does not change.
        """
