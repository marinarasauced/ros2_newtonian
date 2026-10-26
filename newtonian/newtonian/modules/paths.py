
from __future__ import annotations

from pathlib import Path


def resolve(uri: str) -> str:
    """
    Returns the file path of a 'package://', 'newton://', or normal path.
    """
    if uri.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory
        pkg, _, rel = uri[len("package://"):].partition("/")
        return str(Path(get_package_share_directory(pkg)) / rel)
    if uri.startswith("newton://"):
        import newton.examples
        return newton.examples.get_asset(uri[len("newton://"):])
    return str(Path(uri).expanduser())
