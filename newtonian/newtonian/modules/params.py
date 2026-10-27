
from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any

import yaml


_MISSING = object()


def flatten(tree: dict, prefix: str = "") -> dict[str, Any]:
    """
    Returns 'tree' as one flat dict with dotted keys.
    """
    out: dict[str, Any] = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            out.update(flatten(value, name + "."))
        else:
            out[name] = value
    return out


class ParameterError(ValueError):
    """
    Raised when a configuration key is missing or has a bad value.
    """


class Parameters:
    """
    Flat configuration tree with typed getters that name the full key in errors.
    """

    def __init__(self, flat: dict[str, Any] | None = None, prefix: str = "") -> None:
        """
        Stores the flat tree and the key prefix used in errors.
        """
        self._flat = dict(flat or {})
        self.prefix = prefix

    @classmethod
    def from_dict(cls, tree: dict) -> Parameters:
        """
        Builds flattened tree from dictionary.
        """
        return cls(flatten(tree=tree))

    @classmethod
    def from_yaml(cls, path: str | Path, node_name: str = "placeholder") -> Parameters:
        """
        Builds flattened tree from yaml.
        """
        data = yaml.safe_load(Path(path).read_text())
        for key in (node_name, f"/{node_name}", "/**"):
            if key in data:
                data = data[key]
                break
        return cls.from_dict(data.get("ros__parameters", data))

    def children(self) -> list[str]:
        """
        Returns the names of the direct subtrees in file order.
        """
        seen: dict[str, None] = {}
        for key in self._flat:
            if "." in key:
                seen.setdefault(key.split(".", 1)[0], None)
        return list(seen)

    def check_keys(self, allowed: Iterable[str]) -> None:
        """
        Raises 'ParameterError' if a top-level key is not in 'allowed'.
        """
        allowed = set(allowed)
        unknown = sorted({key.split(".", 1)[0] for key in self._flat} - allowed)
        if unknown:
            raise ParameterError(f"{self.prefix}{unknown[0]} encountered unknown key after expecting one of {sorted(allowed)}")

    def entries(self, group: str, key: str = "type") -> list[tuple[str, Parameters]]:
        """
        Returns the named entries of 'group' that have 'key', or the entries listed in 'names'.
        """
        tree = self.subtree(group)
        names = tree.get("names", None)
        if names is None:
            names = [c for c in tree.children() if key in tree.subtree(c)]
        return [(name, tree.subtree(name)) for name in names]

    def get(self, key: str, default: Any = _MISSING) -> Any:
        """
        Returns the value of 'key', and raises 'ParameterError' if 'key' is missing.
        """
        value = self._flat.get(key, default)
        if value is _MISSING:
            where = f"{self.prefix}{key}"
            raise ParameterError(f"{where} is required but missing")
        if isinstance(value, tuple):
            value = list(value)
        return value

    def get_int(self, key: str, default: Any = _MISSING, at_least: int | None = None) -> int:
        """
        Returns the value of 'key' as an integer, and raises 'ParameterError' if it is not an integer or is out of range.
        """
        value = self.get(key=key, default=default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
            raise ParameterError(f"{self.prefix}{key} expected an integer but got {value!r}")
        if at_least is not None and value < at_least:
            raise ParameterError(f"{self.prefix}{key} expected a value at least {at_least} but got {value!r}")
        return int(value)

    def get_float(self, key: str, default: Any = _MISSING, above: float | None = None, at_least: float | None = None) -> float:
        """
        Returns the value of 'key' as a float, and raises 'ParameterError' if it is not a number or is out of range.
        """
        value = self.get(key=key, default=default)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParameterError(f"{self.prefix}{key} expected a number but got {value!r}")
        if above is not None and not value > above:
            raise ParameterError(f"{self.prefix}{key} expected a value above {above} but got {value!r}")
        if at_least is not None and not value >= at_least:
            raise ParameterError(f"{self.prefix}{key} expected a value at least {at_least} but got {value!r}")
        return float(value)

    def get_choice(self, key: str, choices: Iterable[str], default: Any = _MISSING) -> str:
        """
        Returns the value of 'key', and raises 'ParameterError' if it is not in 'choices'.
        """
        value = self.get(key=key, default=default)
        choices = tuple(choices)
        if value not in choices:
            raise ParameterError(f"{self.prefix}{key} expected one of {list(choices)} but got {value!r}")
        return value

    def get_bool(self, key: str, default: Any = _MISSING) -> bool:
        """
        Returns the value of 'key', and raises 'ParameterError' if it is not true or false.
        """
        value = self.get(key=key, default=default)
        if not isinstance(value, bool):
            raise ParameterError(f"{self.prefix}{key} expected true or false but got {value!r}")
        return value

    def subtree(self, key: str) -> Parameters:
        """
        Returns the subtree of 'key'.
        """
        head = key + "."
        flat = {key[len(head):]: value for key, value in self._flat.items() if key.startswith(head)}
        prefix = self.prefix + head
        return Parameters(flat=flat, prefix=prefix)

    def as_kwargs(self) -> dict[str, Any]:
        """
        Returns the tree as nested dicts.
        """
        out: dict[str, Any] = {}
        for key, value in self._flat.items():
            node = out
            *parents, pear = key.split(".")
            for p in parents:
                node = node.setdefault(p, {})
            node[pear] = list(value) if isinstance(value, tuple) else value
        return out

    def __contains__(self, key: str) -> bool:
        return key in self._flat
