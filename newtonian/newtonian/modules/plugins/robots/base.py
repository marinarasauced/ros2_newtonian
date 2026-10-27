
from __future__ import annotations

from fnmatch import fnmatchcase
from typing import TYPE_CHECKING, Any

import newton
import numpy as np
import warp as wp

from newtonian.modules.arrays import require
from newtonian.modules.params import ParameterError, Parameters
from newtonian.modules.paths import resolve
from newtonian.modules.registry import Registry
from newtonian.modules.transforms import convert_pose_to_transform

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


ROBOTS: Registry = Registry("robot")


DRIVE_MODES = {
    "none": newton.JointTargetMode.NONE,
    "position": newton.JointTargetMode.POSITION,
    "velocity": newton.JointTargetMode.VELOCITY,
    "position_velocity": newton.JointTargetMode.POSITION_VELOCITY,
    "effort": newton.JointTargetMode.EFFORT,
}

JOINT_TYPES = (int(newton.JointType.REVOLUTE), int(newton.JointType.PRISMATIC))

_DRIVE_FIELDS = {
    "ke": "joint_target_ke",
    "kd": "joint_target_kd",
    "effort_limit": "joint_effort_limit",
    "velocity_limit": "joint_velocity_limit",
    "armature": "joint_armature",
    "damping": "joint_damping",
    "friction": "joint_friction",
}



# --- Robot Plugins --------------------------------------------------------------


class Robot(Plugin):
    """
    Base of the robot plugins, which load one robot file with its drives.
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        Starts with no lazy drives.
        """
        super().__init__(name=name, p=p)

        self._lazy_dofs = np.zeros(0, dtype=np.int32)
        self._lazy_gains = np.zeros((0, 2), dtype=np.float32)
        self._drives_active = True
        self._sim: NewtonianSimulator | None = None

    def load(self, builder: newton.ModelBuilder, path: str, xform: wp.transform, floating: bool, args: dict[str, Any]) -> None:
        """
        Loads the robot file into 'builder'.
        """
        raise NotImplementedError

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        Loads the robot, and applies the configs, armature, colors, drives, and start positions.
        """
        super().on_build(sim=sim, builder=builder)
        p = self.p

        robot_builder = newton.ModelBuilder(up_axis=builder.up_axis, gravity=builder.gravity)
        sim.solver_plugin.before_build(robot_builder)
        _configure(cfg=robot_builder.default_joint_cfg, values=p.subtree("joint_cfg").as_kwargs(), where=f"{p.prefix}joint_cfg")
        _configure(cfg=robot_builder.default_shape_cfg, values=p.subtree("shape_cfg").as_kwargs(), where=f"{p.prefix}shape_cfg")

        self.load(
            builder=robot_builder,
            path=resolve(p.get(key="file")),
            xform=convert_pose_to_transform(pose=p.get(key="pose", default=None), where=f"{p.prefix}pose"),
            floating=p.get_bool(key="floating", default=True),
            args=p.subtree("loader_args").as_kwargs(),
        )

        body_armature = p.get_float(key="body_armature", default=0.0, at_least=0.0)
        if body_armature > 0.0:
            for b in range(robot_builder.body_count):
                inertia = np.asarray(robot_builder.body_inertia[b], dtype=np.float32).reshape(3, 3)
                robot_builder.body_inertia[b] = wp.mat33(inertia + np.eye(3, dtype=np.float32) * body_armature)

        apply_colors(builder=robot_builder, colors=p.subtree("colors"))
        lazy = apply_drives(builder=robot_builder, drives=p.subtree("drives"))
        apply_initial_joint_q(builder=robot_builder, values=p.subtree("initial_joint_q").as_kwargs(), where=f"{p.prefix}initial_joint_q")

        dof_offset = builder.joint_dof_count
        builder.add_builder(builder=robot_builder, label_prefix=self.name)
        self._lazy_dofs = np.array([dof_offset + d for d in lazy], dtype=np.int32)
        self._lazy_gains = np.array(list(lazy.values()), dtype=np.float32).reshape(-1, 2)
        self._drives_active = not len(self._lazy_dofs)

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Finds the joints, bodies, and shapes of the robot in the model.
        """
        super().on_model(sim=sim)
        prefix = self.name + "/"

        def own(labels: list[str]) -> dict[int, str]:
            return {i: shorthand(label=label) for i, label in enumerate(labels) if label.startswith(prefix)}

        model = sim.model
        self.joints: dict[int, str] = own(model.joint_label)
        self.bodies: dict[int, str] = own(model.body_label)
        self.shapes: dict[int, str] = own(model.shape_label)
        if not self.bodies:
            raise ValueError(f"robot '{self.name}' has no bodies in the model")
        self.root = min(self.bodies)
        self._sim = sim

    def reset(self) -> None:
        """
        Turns the lazy drives off again.
        """
        super().reset()
        if self._sim is not None and self._drives_active and len(self._lazy_dofs):
            self._set_lazy_gains(sim=self._sim, gains=np.zeros_like(self._lazy_gains))
            self._drives_active = False

    @property
    def has_lazy_drives(self) -> bool:
        """
        True if a drive is lazy.
        False otherwise.
        """
        return bool(len(self._lazy_dofs))

    def activate(self) -> None:
        """
        Turns the lazy drives on.
        """
        if self._sim is not None and not self._drives_active:
            self._set_lazy_gains(sim=self._sim, gains=self._lazy_gains)
            self._drives_active = True

    def find_joints(self, pattern: str | list[str]) -> list[int]:
        """
        Returns the joints that match 'pattern', and raises 'ParameterError' if none match.
        """
        return self._find(table=self.joints, pattern=pattern, what="joint")

    def find_bodies(self, pattern: str | list[str]) -> list[int]:
        """
        Returns the bodies that match 'pattern', and raises 'ParameterError' if none match.
        """
        return self._find(table=self.bodies, pattern=pattern, what="body")

    def find_body(self, name: str | None) -> int:
        """
        Returns the body named 'name', or the root body if 'name' is not set.
        """
        return self.root if not name else self.find_bodies(pattern=name)[0]

    def find_shapes(self, pattern: str | list[str]) -> list[int]:
        """
        Returns the shapes and sites that match 'pattern', and raises 'ParameterError' if none match.
        """
        return self._find(table=self.shapes, pattern=pattern, what="shape/site")

    def _find(self, table: dict[int, str], pattern: str | list[str], what: str) -> list[int]:
        """
        Returns the indices in 'table' that match 'pattern', and raises 'ParameterError' if none match.
        """
        found = [i for i, name in table.items() if matches(name=name, pattern=pattern)]
        if not found:
            raise ParameterError(f"robot '{self.name}' has no {what} matching {pattern!r}")
        return found

    def _set_lazy_gains(self, sim: NewtonianSimulator, gains: np.ndarray) -> None:
        """
        Writes 'gains' to the lazy drives, and notifies the solver.
        """
        model = sim.model
        for col, arr in enumerate((require(arr=model.joint_target_ke), require(arr=model.joint_target_kd))):
            values = arr.numpy()
            values[self._lazy_dofs] = gains[:, col]
            arr.assign(values)
        sim.solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)


class Robots(dict[str, Robot]):
    """
    Robots of a scene as a normal dict where the robot's name keys its instance.
    An additional function 'resolve' finds the robot that a plugin's 'robot' key refers to.
    """

    def resolve(self, name: str | None, caller: str) -> Robot:
        """
        Returns the robot object that a plugin's 'robot' key refers to.
        The 'caller' argument refers to the plugin asking to resolve.
        """
        if name:
            if name not in self:
                raise ParameterError(f"{caller} : unknown robot '{name}' w.r.t. expected {sorted(self)}")
            return self[name]
        if not self:
            raise ParameterError(f"{caller} : the scene has no robot")
        if len(self) > 1:
            raise ParameterError(f"{caller} : set 'robot' to one of {sorted(self)}")
        return next(iter(self.values()))



# --- Helper Functions -----------------------------------------------------------


def shorthand(label: str) -> str:
    """
    Returns shorthand name of 'label'.
    """
    return label.rsplit("/", 1)[-1]


def matches(name: str, pattern: str | list[str]) -> bool:
    """
    True if 'name' matches any glob in 'pattern'.
    False otherwise.
    """
    patterns = [pattern] if isinstance(pattern, str) else pattern
    return any(fnmatchcase(name, p) for p in patterns)


def match(names: list[str], pattern: str | list[str]) -> list[int]:
    """
    Returns indices of 'names' whose shorthand matches any glob in 'pattern'.
    """
    return [i for i, n in enumerate(names) if matches(name=shorthand(label=n), pattern=pattern)]


def apply_colors(builder: newton.ModelBuilder, colors: Parameters) -> None:
    """
    Sets the colors of the shapes that each group selects.
    """
    for group in colors.children():
        c = colors.subtree(group)
        shapes = match(names=builder.shape_label, pattern=c.get(key="shapes"))
        if not shapes:
            raise ParameterError(f"{c.prefix}shapes {c.get(key='shapes')!r} matches no shape")
        color = c.get(key="color")
        if not isinstance(color, list) or len(color) != 3 or not all(isinstance(v, (int, float)) for v in color):
            raise ParameterError(f"{c.prefix}color expected [r, g, b] but got {color!r}")
        for s in shapes:
            builder.shape_color[s] = (float(color[0]), float(color[1]), float(color[2]))


def apply_drives(builder: newton.ModelBuilder, drives: Parameters) -> dict[int, tuple[float, float]]:
    """
    Sets the drive values of the joints that each group selects, and returns the gains of the lazy drives.
    """
    lazy: set[int] = set()
    for drive in drives.children():
        d = drives.subtree(drive)
        joints = match(names=builder.joint_label, pattern=d.get(key="joints"))
        if not joints:
            raise ParameterError(f"{d.prefix}joints {d.get(key='joints')!r} matches no joint")
        mode = d.get_choice(key="mode", choices=DRIVE_MODES) if "mode" in d else None
        is_lazy = d.get_bool(key="lazy") if "lazy" in d else None
        for j in joints:
            for dof in _dofs(builder=builder, joint=j):
                if is_lazy is not None:
                    (lazy.add if is_lazy else lazy.discard)(dof)
                if mode is not None:
                    builder.joint_target_mode[dof] = int(DRIVE_MODES[mode])
                for key, field in _DRIVE_FIELDS.items():
                    if key in d:
                        getattr(builder, field)[dof] = d.get_float(key=key, at_least=0.0)
    gains: dict[int, tuple[float, float]] = {}
    for dof in sorted(lazy):
        gains[dof] = (builder.joint_target_ke[dof], builder.joint_target_kd[dof])
        builder.joint_target_ke[dof] = 0.0
        builder.joint_target_kd[dof] = 0.0
    return gains


def apply_initial_joint_q(builder: newton.ModelBuilder, values: dict[str, float], where: str) -> None:
    """
    Sets the start position and target of the joints that each glob selects.
    """
    for pattern, value in values.items():
        joints = match(names=builder.joint_label, pattern=pattern)
        if not joints:
            raise ParameterError(f"{where} {pattern!r} matches no joint")
        for j in joints:
            q = builder.joint_q_start[j]
            builder.joint_q[q] = float(value)
            builder.joint_target_q[q] = float(value)


def _dofs(builder: newton.ModelBuilder, joint: int) -> range:
    """
    Returns the DOF indices of 'joint'.
    """
    start = builder.joint_qd_start[joint]
    return range(start, start + sum(builder.joint_dof_dim[joint]))


def _configure(cfg: Any, values: dict[str, Any], where: str) -> None:
    """
    Sets the fields of 'cfg', and raises 'ParameterError' for an unknown field.
    """
    for key, value in values.items():
        if not hasattr(cfg, key):
            raise ParameterError(f"{where} has no field '{key}'")
        setattr(cfg, key, value)
