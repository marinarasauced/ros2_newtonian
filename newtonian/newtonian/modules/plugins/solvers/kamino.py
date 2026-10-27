
from __future__ import annotations

import newton

from newtonian.modules.plugins.solvers.base import SOLVERS, Solver


@SOLVERS.register("kamino")
class SolverKamino(Solver):
    """
    Newton Kamino solver.
    """
    solver_cls = newton.solvers.SolverKamino

    def before_build(self, builder: newton.ModelBuilder) -> None:
        """
        Registers the Kamino attributes on the builder.
        """
        newton.solvers.SolverKamino.register_custom_attributes(builder=builder)

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        """
        Returns the Kamino solver, with dict arguments turned into config objects.
        """
        defaults = self.solver_cls.Config(use_collision_detector=True, use_fk_solver=True)
        args = {key: type(getattr(defaults, key))(**value) if isinstance(value, dict) else value for key, value in self.args.items()}
        return self.solver_cls(model, config=self.solver_cls.Config.from_model(model, **args))

    @property
    def reports_contact_forces(self) -> bool:
        """
        True if contacts come from Newton's collision pipeline.
        False otherwise.
        """
        return self.uses_collision_pipeline

    @property
    def supports_lazy_drives(self) -> bool:
        """
        False; lazy drives are not supported.
        """
        return False

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        True unless 'use_collision_detector' is true.
        """
        return not self.args.get("use_collision_detector", False)
