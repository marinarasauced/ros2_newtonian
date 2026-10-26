
from __future__ import annotations

import newton

from newtonian.modules.plugins.solvers.base import SOLVERS, Solver


@SOLVERS.register("xpbd")
class SolverXPBD(Solver):
    """
    Newton XPBD solver.
    """
    solver_cls = newton.solvers.SolverXPBD

    @property
    def reports_contact_forces(self) -> bool:
        """
        True; XPBD gives contact forces.
        """
        return True
