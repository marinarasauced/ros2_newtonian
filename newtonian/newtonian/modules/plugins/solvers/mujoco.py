
from __future__ import annotations

import newton

from newtonian.modules.plugins.solvers.base import SOLVERS, Solver


@SOLVERS.register("mujoco")
class SolverMuJoCo(Solver):
    """
    Newton MuJoCo solver.
    """
    solver_cls = newton.solvers.SolverMuJoCo

    def before_build(self, builder: newton.ModelBuilder) -> None:
        """
        Registers the MuJoCo attributes on the builder.
        """
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder=builder)

    @property
    def reports_actuator_forces(self) -> bool:
        """
        True; MuJoCo gives actuator forces.
        """
        return True

    @property
    def reports_contact_forces(self) -> bool:
        """
        True; MuJoCo gives contact forces.
        """
        return True

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        True if 'use_mujoco_contacts' is false.
        """
        return not self.args.get("use_mujoco_contacts", True)
