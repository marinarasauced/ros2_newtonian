
from __future__ import annotations

from typing import Any

import newton

from newtonian.modules.registry import Registry


SOLVERS: Registry = Registry("solver")



# --- Solver Plugins -------------------------------------------------------------


class Solver:
    """
    Plugin for building and using one Newton solver type.
    """
    solver_cls: type

    def __init__(self, args: dict[str, Any]) -> None:
        """
        Stores the arguments for the Newton solver.
        """
        self.args = args

    def before_build(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before any assets are loaded.
        """

    def before_finalize(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before finalize.
        """

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        """
        Returns the Newton solver for 'model'.
        """
        return self.solver_cls(model, **self.args)

    @property
    def reports_contact_forces(self) -> bool:
        """
        True if the solver can give contact forces.
        False otherwise.
        """
        return False

    @property
    def reports_actuator_forces(self) -> bool:
        """
        True if the solver can give actuator forces.
        False otherwise.
        """
        return False

    @property
    def supports_lazy_drives(self) -> bool:
        """
        True if the solver can change drive gains at runtime.
        False otherwise.
        """
        return True

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        True if contacts come from Newton's collision pipeline ;
        False if the solver must compute its own contacts.
        """
        return True

    def update_contacts(self, solver: newton.solvers.SolverBase, contacts: newton.Contacts, state: newton.State) -> None:
        """
        Writes contact forces into the buffer after a frame.
        """
        solver.update_contacts(contacts, state)
