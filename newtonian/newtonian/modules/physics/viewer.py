
from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

import queue
import threading
import traceback

import newton
import numpy as np
import warp as wp

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


_PANEL_MARGIN = 10
_PANEL_ALPHA = 0.7
_SKY_COLOR = (38.0 / 255.0, 36.0 / 255.0, 38.0 / 255.0)

_STREAM_LOCK = threading.RLock()
_streams_serialized = False


def _serialize_streams() -> None:
    """
    Makes the scoped stream blocks of Warp atomic between threads.
    """
    global _streams_serialized
    if _streams_serialized:
        return
    enter, leave = wp.ScopedStream.__enter__, wp.ScopedStream.__exit__

    def locked_enter(self):
        """
        """
        _STREAM_LOCK.acquire()
        try:
            return enter(self)
        except BaseException:
            _STREAM_LOCK.release()
            raise

    def locked_exit(self, exc_type, exc_value, traceback):
        try:
            return leave(self, exc_type, exc_value, traceback)
        finally:
            _STREAM_LOCK.release()

    wp.ScopedStream.__enter__, wp.ScopedStream.__exit__ = locked_enter, locked_exit
    _streams_serialized = True


class _ViewerHost(Protocol):
    """
    """

    @property
    def model(self) -> newton.Model: ...

    def pause(self, is_paused: bool) -> None: ...


# either viewer
type Viewer = NewtonianViewer | NewtonianViewerThread


class NewtonianViewer:
    """
    Newton GL viewer with a control panel, kept across rebuilds.
    """

    def __init__(self, sim: _ViewerHost) -> None:
        """
        Opens the viewer window for 'sim'.
        """
        self.gl = newton.viewer.ViewerGL()
        self._reset_requested = False
        self._render_setup()
        self._ui_setup()
        self.attach(sim=sim)

    def attach(self, sim: _ViewerHost) -> NewtonianViewer:
        """
        Shows the model of 'sim', and returns the viewer.
        """
        self.sim = sim
        self.gl.set_model(model=sim.model)
        # self._render_planes()
        # register custom ui elements upon model changes
        if self.gl.gui is not None:
            self.gl.register_ui_callback(callback=self._ui_control, position="free")
        return self

    @property
    def is_paused(self) -> bool:
        """
        """
        return self.gl._paused

    @is_paused.setter
    def is_paused(self, is_paused: bool) -> None:
        """
        True if stepping is paused.
        False otherwise.
        """
        self.gl._paused = is_paused

    def is_running(self) -> bool:
        """
        True if the window is open.
        False otherwise.
        """
        return self.gl.is_running()

    def stepable(self) -> bool:
        """
        Returns whether the simulator should step.
        """
        return self.gl.should_step()

    def render(self, time: float, state: newton.State, contacts: newton.Contacts | None) -> None:
        """
        Renders one frame for the simulator's Newton GL viewer.
        """
        self.gl.begin_frame(time=time)
        self.gl.log_state(state=state)
        if contacts is not None:
            self.gl.log_contacts(contacts=contacts, state=state)
        self.gl.end_frame()

    def draw_lines(self, name: str, starts: np.ndarray | None, ends: np.ndarray | None, colors: np.ndarray | None, device: wp.Device, width: float = 1.5) -> None:
        """
        Draws line segments from 'starts' to 'ends' (N, 3) with 'colors' (N, 3) as the batch 'name', or hides the batch when they are 'None'. The viewer has one line width in pixels for all batches.
        """
        if starts is None or ends is None or colors is None:
            self.gl.log_lines(name=name, starts=None, ends=None, colors=None)
            return
        self.gl.renderer.line_width = width
        self.gl.log_lines(
            name=name,
            starts=wp.array(starts, dtype=wp.vec3, device=device),
            ends=wp.array(ends, dtype=wp.vec3, device=device),
            colors=wp.array(colors, dtype=wp.vec3, device=device),
        )

    def close(self) -> None:
        """
        Closes the simulator's Newton GL viewer.
        """
        self.gl.close()

    def apply(self, state: newton.State) -> None:
        """
        Applies forces in the simulator's Newton GL viewer.
        """
        self.gl.apply_forces(state=state)

    def take_reset_request(self) -> bool:
        """
        True once if the reset buttons was pressed since the last call.
        """
        requested, self._reset_requested = self._reset_requested, False
        return requested

    def _render_setup(self) -> None:
        """
        Sets the sky to one flat color.
        """
        renderer = self.gl.renderer
        renderer.sky_upper = _SKY_COLOR
        renderer.sky_lower = _SKY_COLOR

    # def _render_planes(self) -> None:
    #     """
    #     Removes the checker pattern from planes.
    #     """
    #     for batch in self.gl._shape_instances.values():
    #         if batch.geo_type == newton.GeoType.PLANE:
    #             materials = batch.materials.numpy()
    #             materials[:, 2] = 0.0
    #             batch.materials.assign(materials)

    def _ui_setup(self) -> None:
        """
        Sets up custom ui elements.
        """
        viewer = self.gl
        viewer.wind = None
        if viewer.gui is None:
            return
        self._ui_stats = (0.0, 0.0)
        viewer.gui._render_left_panel = lambda: None    # type: ignore[method-assign]
        viewer.gui.is_capturing = lambda: viewer.gui.is_available and bool(viewer.gui.ui.io.want_text_input)    # type: ignore[method-assign]
        viewer.register_ui_callback(callback=self._ui_measure, position="stats")

    def _ui_measure(self, imgui) -> None:
        """
        Measures and writes ui position and size to stats attribute.
        """
        pos, size = imgui.get_window_pos(), imgui.get_window_size()
        self._ui_stats = (pos.y + size.y, size.x)

    def _ui_control(self, imgui) -> None:
        """
        Custom ui elements for simulator control.
        """
        viewer = self.gl
        gui = viewer.gui
        if gui is None:
            return

        scale = gui.ui.dpi_scale
        bottom, width = self._ui_stats

        # button size
        size = imgui.ImVec2(width - 2 * imgui.get_style().window_padding.x, 0)

        # button placement
        imgui.set_next_window_pos(imgui.ImVec2(imgui.get_io().display_size.x - _PANEL_MARGIN * scale, bottom + _PANEL_MARGIN * scale), pivot=imgui.ImVec2(1.0, 0.0))
        imgui.set_next_window_bg_alpha(_PANEL_ALPHA)

        # overlay flags
        flags = (imgui.WindowFlags_.no_decoration.value | imgui.WindowFlags_.always_auto_resize.value | imgui.WindowFlags_.no_saved_settings.value | imgui.WindowFlags_.no_focus_on_appearing.value | imgui.WindowFlags_.no_nav.value | imgui.WindowFlags_.no_move.value)

        # overlay
        expanded, _ = imgui.begin("Simulation Controls", flags=flags)
        if expanded:

            # pause
            if imgui.button("Resume" if self.is_paused else "Pause", size):
                self.sim.pause(is_paused=not self.is_paused)

            # step
            imgui.begin_disabled(not self.is_paused)
            if imgui.button("Step", size):
                viewer._step_requested = True
            imgui.end_disabled()

            # reset
            if imgui.button("Reset", size):
                self._reset_requested = True

        imgui.end()


class _NewtonianViewerSimulator:
    """
    """

    def __init__(self, owner: NewtonianViewerThread) -> None:
        """
        """
        self._owner = owner

    @property
    def model(self) -> newton.Model:
        """
        """
        return self._owner.sim.model

    def pause(self, is_paused: bool) -> None:
        """
        """
        self._owner.sim.pause(is_paused=is_paused)


class NewtonianViewerThread:
    """
    """

    def __init__(self, sim: NewtonianSimulator) -> None:
        """
        """
        _serialize_streams()
        self.sim = sim
        self._real: NewtonianViewer
        self._error: BaseException | None = None
        self._stop = False
        self._snapshot: newton.State | None = None
        self._job: tuple[float, newton.State, newton.Contacts | None] | None = None
        self._busy = False
        self._wake = threading.Event()
        self._ready = threading.Event()
        self._calls: queue.SimpleQueue = queue.SimpleQueue()
        self._thread = threading.Thread(target=self._run, name="newtonian-viewer", daemon=True)
        self._thread.start()
        self._ready.wait()
        if self._error is not None:
            raise RuntimeError("the viewer thread could not open the window") from self._error

    def attach(self, sim: NewtonianSimulator) -> NewtonianViewerThread:
        """
        """
        def attach() -> None:
            self._job, self._busy, self._snapshot = None, False, None
            self._real.attach(sim=_NewtonianViewerSimulator(self))
        
        self.sim = sim
        self._call(fn=attach)
        return self

    @property
    def is_paused(self) -> bool:
        """
        True if stepping is paused.
        False otherwise.
        """
        return self._real.is_paused

    @is_paused.setter
    def is_paused(self, is_paused: bool) -> None:
        """
        Pauses or unpauses stepping.
        """
        self._real.is_paused = is_paused

    def is_running(self) -> bool:
        """
        True if the window is open and the thread is running.
        False otherwise.
        """
        return self._error is None and self._thread.is_alive() and self._real.is_running()

    def stepable(self) -> bool:
        """
        Returns whether the simulator should step.
        """
        return self._real.stepable()

    def take_reset_request(self) -> bool:
        """
        True once if the reset button was pressed since the last call.
        """
        return self._real.take_reset_request()

    def render(self, time: float, state: newton.State, contacts: newton.Contacts | None) -> None:
        """
        Hands a copy of 'state' to the viewer thread, unless it is still drawing the last frame.
        """
        if self._busy or not self.is_running():
            return
        if self._snapshot is None:
            self._snapshot = self.sim.model.state()
        self._snapshot.assign(state)
        self._job = (time, self._snapshot, contacts)
        self._busy = True
        self._wake.set()

    def draw_lines(self, name: str, starts: np.ndarray | None, ends: np.ndarray | None, colors: np.ndarray | None, device: wp.Device, width: float = 1.5) -> None:
        """
        Drarws line segments in the viewer thread, or hides the batch when they are 'None'.
        """
        self._call(fn=lambda: self._real.draw_lines(name=name, starts=starts, ends=ends, colors=colors, device=device, width=width), wait=False)

    def apply(self, state: newton.State) -> None:
        """
        Applies forces of the viewer in the calling thread.
        """
        self._real.apply(state=state)

    def close(self) -> None:
        """
        Stops the thread, thereby closing the window.
        """
        self._stop = True
        self._wake.set()
        self._thread.join(timeout=10.0)

    def _call(self, fn, wait: bool = True) -> None:
        """
        Runs 'fn' in the viewer thread and waits for it unless 'wait' is false.
        """
        done = threading.Event()
        self._calls.put((fn, done))
        self._wake.set()
        if wait:
            while not done.wait(timeout=0.1):
                if not self._thread.is_alive():
                    break

    def _run(self) -> None:
        """
        Opens the viewer and serves the calls and frames until stopped.
        """
        try:
            self._real = NewtonianViewer(sim=_NewtonianViewerSimulator(self))
        except BaseException as e:
            self._error = e
            self._ready.set()
            return
        self._ready.set()
        try:
            while not self._stop:
                self._wake.wait()
                self._wake.clear()
                while True:
                    try:
                        fn, done = self._calls.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        fn()
                    finally:
                        done.set()
                job, self._job = self._job, None
                if job is not None:
                    self._real.render(time=job[0], state=job[1], contacts=job[2])
                    self._busy = False
        except BaseException as e:
            self._error = e
            traceback.print_exc()
        finally:
            self._busy = False
            try:
                self._real.close()
            except BaseException:
                pass
