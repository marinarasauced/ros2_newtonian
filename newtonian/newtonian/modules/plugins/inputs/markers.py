
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from visualization_msgs.msg import Marker, MarkerArray

from newtonian.modules.params import Parameters
from newtonian.modules.plugins.inputs.base import INPUTS, Input

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator
    from newtonian.modules.physics.viewer import NewtonianViewer


@INPUTS.register("markers")
class InputMarkers(Input):
    """
    Draws the line markers of a 'MarkerArray' topic in the viewer without commanding the robot.
    """
    msg_type = MarkerArray
    default_timeout = 0.5

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Reads the common input keys.
        """
        super().__init__(name=name, p=p)

        self.line_width = p.get_float(key="line_width", default=2.0, above=0.0)
        self._names: set[str] = set()
        self._time_drawn: float | None = None

    def _draw(self, sim: NewtonianSimulator, viewer: NewtonianViewer, msg: MarkerArray) -> None:
        """
        Draws the 'LINE_LIST' and 'LINE_STRIP' markers of 'msg', one batch of lines per namespace and id.
        """
        for marker in msg.markers:
            if marker.type not in (Marker.LINE_LIST, Marker.LINE_STRIP) or not marker.points:
                continue
            points = np.array([(q.x, q.y, q.z) for q in marker.points], dtype=np.float32)
            if marker.colors:
                colors = np.array([(c.r, c.g, c.b) for c in marker.colors], dtype=np.float32)
            else:
                colors = np.tile(np.array([marker.color.r, marker.color.g, marker.color.b], dtype=np.float32), (len(points), 1))
            if marker.type == Marker.LINE_LIST:
                count = len(points) // 2
                starts, ends, colors = points[0:2 * count:2], points[1:2 * count:2], colors[0:2 * count:2]
            else:
                starts, ends, colors = points[:-1], points[1:], colors[:-1]
            name = f"{self.name}/{marker.ns}/{marker.id}"
            self._names.add(name)
            viewer.draw_lines(name=name, starts=starts, ends=ends, colors=colors, device=sim.device, width=self.line_width)

    def control(self, sim: NewtonianSimulator) -> None:
        """
        Draws a new message, and hides the lines when the last message is older than 'timeout'.
        """
        viewer = sim.viewer
        if viewer is None:
            return
        msg, self._msg = self._msg, None
        if msg is not None:
            self._draw(sim=sim, viewer=viewer, msg=msg)
            self._time_drawn = sim.sim_time
        elif self.timeout > 0.0 and self._time_drawn is not None and sim.sim_time - self._time_drawn > self.timeout:
            for name in self._names:
                viewer.draw_lines(name=name, starts=None, ends=None, colors=None, device=sim.device)
            self._time_drawn = None

    def reset(self) -> None:
        """
        Forgets the drawing time.
        """
        super().reset()

        self._time_drawn = None
