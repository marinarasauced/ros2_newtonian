
from __future__ import annotations

from typing import Any


class Plugin:
    """
    Base of the profile and metric plugins.
    """

    def __init__(self, name: str, p: dict[str, Any]) -> None:
        """
        Stores the name and the parameters.
        """
        self.name = name
        self.p = p
