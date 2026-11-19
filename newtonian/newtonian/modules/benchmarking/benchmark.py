
#pyright: reportArgumentType=false
#pyright: reportReturnType=false
#pyright: reportIndexIssue=false
#pyright: reportOperatorIssue=false
#pyright: reportCallIssue=false
#type: ignore

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import math

import newton
import numpy as np
import warp as wp

from newtonian.modules.arrays import require, to_numpy
from newtonian.modules.transforms import quat_mul

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


_SLOTS = 4
_PLUGIN_KEYS = ("latency", "jitter", "drop")


@wp.kernel
def _add_wrench(
        body_f: wp.array[wp.spatial_vector],
        body: wp.array[wp.int32],
        wrench: wp.array[wp.spatial_vector]
    ) -> None:
    i = wp.tid()
    b = body[i]
    if b >= 0:
        wp.atomic_add(body_f, b, wrench[i])


def _quaternion_from_rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    """
    Returns the quaternion of the roll, pitch, and yaw 'rpy'.
    """
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


class NewtonianBenchmarker:
    """
    Benchmark hooks of the simulator.
    """

    def __init__(self, sim: NewtonianSimulator) -> None:
        """
        Allocates the arrays that the captured step reads.
        """
        self.sim = sim

        # pushes: the schedule on the host, the active wrenches on the device
        self.pushes: list[dict[str, Any]] = []
        self._body = wp.array(np.full(_SLOTS, -1, dtype=np.int32), dtype=wp.int32, device=sim.device)
        self._wrench = wp.zeros(_SLOTS, dtype=wp.spatial_vector, device=sim.device)
        self._active: tuple = ()

        # values of the model before the first change
        self._mu0: np.ndarray | None = None
        self._mass0: tuple[float, np.ndarray, float, np.ndarray] | None = None

        # start state that waits for the end of the warm-up
        self._pending_state: dict[str, Any] | None = None

    def apply(self, state: newton.State) -> None:
        """
        Adds the active push wrenches to 'state.body_f'.
        """
        if state.body_f is None:
            return

        wp.launch(_add_wrench, dim=_SLOTS, inputs=[state.body_f, self._body, self._wrench], device=self.sim.device)

    def update(self, t: float) -> None:
        """
        Writes the wrenches that are active at time 't'.
        """
        if self._pending_state is not None:
            state, self._pending_state = self._pending_state, None
            self.set_state(s=state)

        if not self.pushes and not self._active:
            return

        # sum of the wrenches of the pushes that are active, for each body
        sums: dict[int, np.ndarray] = {}
        for p in self.pushes:
            if p["t0"] - 1.0e-9 <= t < p["t1"] - 1.0e-9:
                sums[p["body"]] = sums.get(p["body"], np.zeros(6)) + p["wrench"]

        # write only if the active wrenches changed
        key = tuple((b, tuple(w)) for b, w in sorted(sums.items()))
        if key == self._active:
            return
        self._active = key

        body = np.full(_SLOTS, -1, dtype=np.int32)
        wrench = np.zeros((_SLOTS, 6), dtype=np.float32)
        for i, (b, w) in enumerate(sorted(sums.items())[:_SLOTS]):
            body[i] = b
            wrench[i] = w

        self._body.assign(body)
        self._wrench.assign(wrench)

    def reset(self) -> None:
        """
        Clears the pushes.
        """
        self.pushes = []
        self._pending_state = None
        self._active = ()
        self._body.assign(np.full(_SLOTS, -1, dtype=np.int32))
        self._wrench.zero_()

    def configure(self, request: dict[str, Any]) -> dict[str, Any]:
        """
        Applies the keys of 'request'.
        """
        sim = self.sim
        out: dict[str, Any] = {}

        # scene
        if request.get("reset"):
            sim.reset()
        if "seed" in request:
            sim.rng = np.random.default_rng(int(request["seed"]))
        if request.get("defaults"):
            self.restore_defaults()

        # physics
        if "friction" in request:
            self.set_friction(mu=float(request["friction"]))
        if "payload_mass" in request:
            self.set_payload(mass=float(request["payload_mass"]))

        # start state: after the warm-up, if the scene is still warming up
        if "state" in request:
            if sim.warmingup:
                self._pending_state = request["state"]
            else:
                self.set_state(s=request["state"])

        # disturbances
        for push in request.get("push", []):
            self.add_push(p=push)

        # latency of the inputs and of the sensors
        for section, plugins in (("inputs", sim.inputs), ("sensors", sim.sensors)):
            for name, values in request.get(section, {}).items():
                targets = [plugin for plugin in plugins if name == "*" or plugin.name == name]
                if not targets:
                    raise ValueError(f"no {section[:-1]} named {name!r}")

                for plugin in targets:
                    for key in _PLUGIN_KEYS:
                        if key in values:
                            setattr(plugin, key, float(values[key]))

        # query
        if "query" in request:
            out.update(self.describe(what=request["query"]))

        return out

    def describe(self, what: str) -> dict[str, Any]:
        """
        Returns facts about the scene.
        """
        sim = self.sim
        if what != "info":
            raise ValueError(f"unknown query {what!r}")

        return {
            "frame_rate": sim.frame_rate,
            "substeps": sim.substeps,
            "root_body": int(next(iter(sim.robots.values())).root),
            "sim_time": sim.sim_time,
            "frame": sim.frame,
            "warming": bool(sim.warmingup),
            "mass": float(to_numpy(arr=sim.model.body_mass).sum()),
            "gravity": float(abs(sim.gravity)),
        }

    def restore_defaults(self) -> None:
        """
        Restores the friction, the payload, and the latency.
        """
        if self._mu0 is not None:
            require(arr=self.sim.model.shape_material_mu).assign(self._mu0)
            self.sim.solver.notify_model_changed(newton.ModelFlags.SHAPE_PROPERTIES)

        if self._mass0 is not None:
            self.set_payload(mass=0.0)

        for plugin in (*self.sim.inputs, *self.sim.sensors):
            for key in _PLUGIN_KEYS:
                if hasattr(plugin, key):
                    setattr(plugin, key, 0.0)

    def set_friction(self, mu: float) -> None:
        """
        Sets the friction of all shapes.
        """
        arr = require(arr=self.sim.model.shape_material_mu)
        if self._mu0 is None:
            self._mu0 = arr.numpy().copy()

        arr.assign(np.full_like(self._mu0, mu))
        self.sim.solver.notify_model_changed(newton.ModelFlags.SHAPE_PROPERTIES)

    def set_payload(self, mass: float) -> None:
        """
        Adds 'mass' to the root body.
        """
        model = self.sim.model
        root = next(iter(self.sim.robots.values())).root

        m = to_numpy(arr=model.body_mass)
        inv_m = to_numpy(arr=model.body_inv_mass)
        inertia = to_numpy(arr=model.body_inertia)
        inv_inertia = to_numpy(arr=model.body_inv_inertia)

        # values of the root body before the first change
        if self._mass0 is None:
            self._mass0 = (float(m[root]), inertia[root].copy(), float(inv_m[root]), inv_inertia[root].copy())
        m0, inertia0, inv_m0, inv_inertia0 = self._mass0

        # mass and inertia scale together
        scale = (m0 + mass) / m0
        m[root] = m0 * scale
        inv_m[root] = inv_m0 / scale
        inertia[root] = inertia0 * scale
        inv_inertia[root] = inv_inertia0 / scale

        require(arr=model.body_mass).assign(m)
        require(arr=model.body_inv_mass).assign(inv_m)
        require(arr=model.body_inertia).assign(inertia)
        require(arr=model.body_inv_inertia).assign(inv_inertia)
        self.sim.solver.notify_model_changed(newton.ModelFlags.BODY_INERTIAL_PROPERTIES)

    def set_state(self, s: dict[str, Any]) -> None:
        """
        Offsets the pose and the velocity of the base, and the joints.
        """
        sim = self.sim
        model = sim.model

        q = to_numpy(arr=model.joint_q).copy()
        qd = to_numpy(arr=model.joint_qd).copy()

        # pose of the floating base: position, then orientation
        if "xyz" in s:
            q[0:3] += np.asarray(s["xyz"], dtype=np.float32)
        if "rpy" in s:
            q[3:7] = quat_mul(a=_quaternion_from_rpy(rpy=tuple(float(v) for v in s["rpy"])), b=q[3:7].astype(np.float64))

        # joints
        if "joint_std" in s:
            q[7:] += sim.rng.normal(0.0, float(s["joint_std"]), size=q[7:].shape).astype(np.float32)
        if "joint_q" in s:
            q[7:] += float(s["joint_q"])

        # velocity of the floating base
        if "v" in s:
            qd[0:3] = np.asarray(s["v"], dtype=np.float32)
        if "w" in s:
            qd[3:6] = np.asarray(s["w"], dtype=np.float32)

        # write to the state
        joint_q = wp.array(q, dtype=wp.float32, device=sim.device)
        joint_qd = wp.array(qd, dtype=wp.float32, device=sim.device)
        with wp.ScopedDevice(device=sim.device):
            newton.eval_fk(model=model, joint_q=joint_q, joint_qd=joint_qd, state=sim.state_0)
            if sim.state_0.joint_q is not None:
                sim.state_0.joint_q.assign(joint_q)
            if sim.state_0.joint_qd is not None:
                sim.state_0.joint_qd.assign(joint_qd)

    def add_push(self, p: dict[str, Any]) -> None:
        """
        Schedules a push.
        """
        sim = self.sim
        body = next(iter(sim.robots.values())).find_body(name=p.get("body"))

        wrench = np.zeros(6)
        wrench[0:3] = p.get("force", [0.0, 0.0, 0.0])
        wrench[3:6] = p.get("torque", [0.0, 0.0, 0.0])

        t0 = sim.sim_time + float(p.get("start", 0.0))
        t1 = t0 + float(p.get("duration", sim.frame_dt))
        self.pushes.append({"t0": t0, "t1": t1, "body": int(body), "wrench": wrench})
