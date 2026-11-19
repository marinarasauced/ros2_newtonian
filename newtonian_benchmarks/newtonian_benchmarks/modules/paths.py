
from __future__ import annotations

from pathlib import Path


# root of the package: the folder that holds 'config/'
ROOT = Path(__file__).resolve().parents[2]


def config_roots() -> list[Path]:
    """
    Returns the folders that can hold the config files.
    """
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(Path(get_package_share_directory("newtonian_benchmarks")) / "config")
    except Exception:
        pass
    roots.append(ROOT / "config")
    return roots


def find_config(name: str) -> Path:
    """
    Returns the path of the config file 'name'.
    """
    path = Path(name)
    if path.is_file():
        return path
    for root in config_roots():
        candidate = root / (name if name.endswith((".yaml", ".yml")) else f"{name}.yaml")
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"no config file '{name}' in {[str(r) for r in config_roots()]}")
