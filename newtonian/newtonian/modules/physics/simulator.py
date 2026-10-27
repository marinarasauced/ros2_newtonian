
from __future__ import annotations

import time

import newton
import numpy as np
import warp as wp

from newtonian.modules.arrays import require, to_numpy
from newtonian.modules.params import ParameterError, Parameters

from newtonian.modules.physics.viewer import NewtonianViewer

from newtonian.modules.plugins.base import Plugin
from newtonian.modules.plugins.environments import ENVIRONMENTS, Environment
from newtonian.modules.plugins.inputs import INPUTS, Input
from newtonian.modules.plugins.robots import ROBOTS, Robots
from newtonian.modules.plugins.sensors import SENSORS, Sensor, update_sensors
from newtonian.modules.plugins.solvers import SOLVERS


class NewtonianSimulatorBuffers:
    """
    Host copies of the joint targets and forces.
    """

    def __init__(self, control: newton.Control) -> None:
        """
        Copies the joint targets and forces of 'control' to the host.
        """
        self.joint_target_q = to_numpy(arr=control.joint_target_q).copy()
        self.joint_target_qd = to_numpy(arr=control.joint_target_qd).copy()
        self.joint_f = to_numpy(arr=control.joint_f).copy()
        self.dirty = False

    def write(self, control: newton.Control) -> None:
        """
        Writes copies of the host arrays into 'control' and clears 'dirty'.
        """
        require(arr=control.joint_target_q).assign(self.joint_target_q)
        require(arr=control.joint_target_qd).assign(self.joint_target_qd)
        require(arr=control.joint_f).assign(self.joint_f)
        self.dirty = False


class NewtonianSimulator:
    """
    Newton scene built from the parameters, with its solver, states, plugins, and viewer.
    """

    def __init__(self, p: Parameters, viewer: NewtonianViewer | None = None) -> None:
        """
        Reads the 'sim' keys and builds the scene.
        """
        self.p = p

        # 'sim' parameters subtree
        p.check_keys(allowed=("sim", "solver", "environments", "robots", "inputs", "sensors"))
        s = p.subtree("sim")
        s.check_keys(allowed=("device", "substeps", "frame_rate", "real_time_factor", "gravity", "viewer", "viewer_rate", "cuda_graph", "seed", "warmup_steps"))

        # warp device
        self.device = wp.get_device(s.get(key="device", default=None))

        # timing
        self.substeps = s.get_int(key="substeps", default=2, at_least=1)
        self.frame_rate = s.get_float(key="frame_rate", default=60.0, above=0.0)
        self.frame_dt = 1.0 / self.frame_rate
        self.sim_dt = self.frame_dt / self.substeps
        self.real_time_factor = s.get_float(key="real_time_factor", default=1.0, above=0.0)
        self.viewer_rate = s.get_float(key="viewer_rate", default=60.0, above=0.0)
        self.warmupsteps = s.get_int(key="warmup_steps", default=0, at_least=0)

        self.gravity = s.get_float(key="gravity", default=-9.81)
        self.viewer_type = s.get_choice(key="viewer", choices=("none", "gl"), default="none")
        self.use_cuda_graph = s.get_bool(key="cuda_graph", default=True)

        # random number generator
        self.rng = np.random.default_rng(s.get_int(key="seed", default=42))

        # build state
        self.frame = 0
        self.graph: wp.Graph | None = None
        self.viewer: NewtonianViewer | None = None
        self.contacts: newton.Contacts | None = None
        self.collision_pipeline: newton.CollisionPipeline | None = None

        # sensors
        self._needs_contacts = False

        # simulation
        self._is_paused = False
        self._warmup_remaining = 0

        # build
        with wp.ScopedDevice(device=self.device):
            self._build(viewer=viewer)

    def _build(self, viewer: NewtonianViewer | None) -> None:
        """
        Builds the model, solver, states, contacts, viewer, and CUDA graph.
        """
        p = self.p

        # setup
        solver_type = p.get_choice(key="solver.type", choices=SOLVERS.names(), default="mujoco")
        self.solver_plugin = SOLVERS.get(solver_type)(p.subtree(f"solver.{solver_type}").as_kwargs())
        scene = newton.ModelBuilder(gravity=(0.0, 0.0, self.gravity))
        self.solver_plugin.before_build(scene)

        # plugins (environments)
        self.environments: list[Environment] = [ENVIRONMENTS.get(q.get(key="type"))(n, q) for n, q in p.entries("environments")]

        # plugins (robots)
        self.robots = Robots((n, ROBOTS.get(q.get(key="type"))(n, q)) for n, q in p.entries("robots"))
        if len(self.robots) > 1:
            raise ParameterError(f"robots has {len(self.robots)} entries but only one robot per sim is supported")

        # plugins (inputs)
        self.inputs: list[Input] = [INPUTS.get(q.get(key="type"))(n, q) for n, q in p.entries("inputs")]

        # plugins (sensors)
        self.sensors: list[Sensor] = [SENSORS.get(q.get(key="type"))(n, q) for n, q in p.entries("sensors")]
        self.sensors += [c for s in self.sensors for c in s.companions()]

        # plugins
        self.plugins: tuple[Plugin, ...] = (*self.environments, *self.robots.values(), *self.inputs, *self.sensors)
        for item in self.plugins:
            item.on_build(sim=self, builder=scene)
        self.solver_plugin.before_finalize(scene)
        self.model = scene.finalize()
        for item in self.plugins:
            item.on_model(sim=self)

        # validation of solvers
        if self._needs_contacts and not self.solver_plugin.reports_contact_forces:
            raise ParameterError(f"solver.type '{solver_type}' does not report contact forces, which contact sensors need")

        if not self.solver_plugin.supports_lazy_drives and any(r.has_lazy_drives for r in self.robots.values()):
            raise ParameterError(f"solver.type '{solver_type}' does not support lazy drives")

        # build solver
        self.solver = self.solver_plugin.create(self.model)

        # build states
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(model=self.model, joint_q=require(self.model.joint_q), joint_qd=require(self.model.joint_qd), state=self.state_0)

        self.buffers = NewtonianSimulatorBuffers(control=self.control)
        self._buffers_initial = (self.buffers.joint_target_q.copy(), self.buffers.joint_target_qd.copy(), self.buffers.joint_f.copy())
        self._state_initial = self.model.state()
        self._state_initial.assign(self.state_0)

        # build contacts
        if self.solver_plugin.uses_collision_pipeline:
            self.collision_pipeline = newton.CollisionPipeline(self.model, **p.subtree("solver.collision_args").as_kwargs())
            self.contacts = self.collision_pipeline.contacts()
        elif self._needs_contacts:
            self.contacts = newton.Contacts(rigid_contact_max=self.solver.get_max_contact_count(), soft_contact_max=0, requested_attributes=self.model.get_requested_contact_attributes())

        # build custom viewer
        if self.viewer_type == "gl":
            self.viewer = viewer.attach(sim=self) if viewer is not None else NewtonianViewer(sim=self)

        # build CUDA graph
        self._capture()

        # warmup
        self._warmup_remaining = self.warmupsteps

    def _capture(self) -> None:
        """
        Captures one frame of '_simulate' as a CUDA graph when the device is CUDA and when 'use_cuda_graph' is true.
        """
        if not (self.use_cuda_graph and self.device.is_cuda):
            return

        # warm up '_simulate' before capture for graph.
        self._simulate()

        # capture '_simulate'.
        with wp.ScopedCapture() as capture:
            self._simulate()
        self.graph = capture.graph

        # restore simulator states to before warm up.
        self._restore()

    def _simulate(self, substeps: int | None = None) -> None:
        """
        Simulates 'substeps' steps of the simulator's physics.
        """
        n = self.substeps if substeps is None else substeps
        for i in range(n):

            # apply forces
            self.state_0.clear_forces()
            if self.viewer is not None:
                self.viewer.apply(state=self.state_0)

            # collision
            if self.collision_pipeline is not None and self.contacts is not None:
                self.collision_pipeline.collide(state=self.state_0, contacts=self.contacts)

            # step physics once
            self.solver.step(state_in=self.state_0, state_out=self.state_1, control=self.control, contacts=self.contacts, dt=self.sim_dt)
            if n % 2 and i == n - 1:
                self.state_0.assign(other=self.state_1)
            else:
                self.state_0, self.state_1 = self.state_1, self.state_0

        # update contacts through plugin
        if self._needs_contacts and self.contacts is not None:
            self.solver_plugin.update_contacts(solver=self.solver, contacts=self.contacts, state=self.state_0)

    def _restore(self) -> None:
        """
        Restores initial state into 'state_0' and 'state_1'.
        """
        for state in (self.state_0, self.state_1):
            for name, arr in vars(state).items():
                if isinstance(arr, wp.array) and getattr(self._state_initial, name, None) is None:
                    setattr(self._state_initial, name, wp.zeros_like(arr))
            state.assign(self._state_initial)

    def close(self) -> None:
        """
        Closes the simulator's Newton GL viewer if 'viewer' exists.
        """
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    @property
    def is_paused(self) -> bool:
        """
        True if stepping is paused.
        False otherwise.
        """
        if self.viewer is not None:
            return self.viewer.is_paused
        return self._is_paused

    def pause(self, is_paused: bool) -> None:
        """
        Pauses or unpauses stepping.
        """
        if self.viewer is not None:
            self.viewer.is_paused = is_paused
        else:
            self._is_paused = is_paused

    def render(self) -> None:
        """
        Renders the current state in the simulator's Newton GL viewer.
        """
        if self.viewer is None:
            return
        self.viewer.render(time=self.sim_time, state=self.state_0, contacts=self.contacts)

    def reset(self) -> None:
        """
        Resets the simulator to its initial states.
        """
        self._restore()
        joint_target_q, joint_target_qd, joint_f = self._buffers_initial
        self.buffers.joint_target_q[:] = joint_target_q
        self.buffers.joint_target_qd[:] = joint_target_qd
        self.buffers.joint_f[:] = joint_f
        self.buffers.write(control=self.control)

        # plugins
        for item in self.plugins:
            item.reset()
        self.frame = 0
        self._warmup_remaining = self.warmupsteps

    def require_contacts(self) -> None:
        """
        Called by contact sensors in 'on_model' to allocate contacts and compute forces.
        """
        self._needs_contacts = True

    def sense(self) -> list[Sensor]:
        """
        Calls sensor updates that are due at current sim time.
        """
        return update_sensors(sim=self, sensors=self.sensors)

    @property
    def sim_time(self) -> float:
        """
        Returns the current time.
        """
        return self.frame * self.frame_dt

    @property
    def state(self) -> newton.State:
        """
        Returns the current state.
        """
        return self.state_0

    def stepable(self) -> bool:
        """
        Returns whether the simulator should step.
        """
        if self.viewer is not None:
            return self.viewer.stepable()
        return not self.is_paused

    def step(self) -> None:
        """
        Steps the physics safely.
        """
        # apply inputs
        for item in self.inputs:
            item.control(sim=self)

        # write targets to device
        if self.buffers.dirty:
            self.buffers.write(control=self.control)

        # simulate one frame
        with wp.ScopedDevice(device=self.device):
            if self.graph is not None:
                wp.capture_launch(graph=self.graph)
            else:
                self._simulate()
        self.frame += 1

    @property
    def warmingup(self) -> bool:
        """
        True if still warming up.
        False otherwise.
        """
        return self._warmup_remaining > 0

    def warmup(self, budget: float) -> None:
        """
        """
        end = time.monotonic() + budget
        with wp.ScopedDevice(device=self.device):
            while self._warmup_remaining > 0 and time.monotonic() < end:
                n = min(self._warmup_remaining, self.substeps)
                if self.graph is not None and n == self.substeps:
                    wp.capture_launch(graph=self.graph)
                else:
                    self._simulate(substeps=n)
                wp.synchronize_device(device=self.device)
                self._warmup_remaining -= n
