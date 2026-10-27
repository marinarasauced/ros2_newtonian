
from __future__ import annotations

from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")


class Registry(Generic[T]):
    """
    Maps a 'type' string from the configuration to a factory.
    """

    def __init__(self, kind: str) -> None:
        """
        Stores the plugin kind used in errors.
        """
        self.kind = kind
        self._items: dict[str, T] = {}

    def register(self, name: str) -> Callable[[T], T]:
        """
        Registers the item under 'name', and raises 'KeyError' if 'name' is already registered.
        """
        def decorator(item: T) -> T:
            if name in self._items:
                raise KeyError(f"{self.kind} type '{name}' is already registered")
            self._items[name] = item
            return item

        return decorator

    def get(self, name: str) -> T:
        """
        Returns the item registered as 'name', and raises 'KeyError' if 'name' is unknown.
        """
        try:
            return self._items[name]
        except KeyError:
            raise KeyError(f"{self.kind} type '{name}' is unknown") from None

    def names(self) -> list[str]:
        """
        Returns the sorted registered names.
        """
        return sorted(self._items)

    def __contains__(self, name: str) -> bool:
        return name in self._items
