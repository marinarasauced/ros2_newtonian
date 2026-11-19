
from __future__ import annotations

from typing import Any

import json
from pathlib import Path

import yaml

from newtonian_benchmarks.modules.paths import find_config


def load_yaml(path_or_name: str) -> dict[str, Any]:
    """
    Returns the content of the config file 'path_or_name'.
    """
    path = find_config(name=path_or_name)
    data = yaml.safe_load(path.read_text())
    data["_path"] = str(path)

    return data


def apply_robot(run: dict[str, Any], profile: dict[str, Any]) -> dict[str, Any]:
    """
    Returns 'run' with the defaults of the robot block under it.
    """
    out = json.loads(json.dumps(run))

    out["robot"] = profile["name"]
    out["trial"] = {"height": profile["height"], **out.get("trial", {})}
    for key in ("thresholds", "pass"):
        out[key] = {**profile.get("defaults", {}).get(key, {}), **out.get(key, {})}
    out["robot_profile"] = profile

    return out


def scene(profile: dict[str, Any]) -> dict[str, Any]:
    """
    Returns the simulator parameters of the robot block as a parameter file tree.
    """
    value = profile["scene"]
    if isinstance(value, str):
        return yaml.safe_load(Path(value).expanduser().read_text())

    return {"newtonian": {"ros__parameters": json.loads(json.dumps(value))}}


def defaults(run: dict[str, Any]) -> dict[str, Any]:
    """
    Returns the keys of 'run' that are not suites.
    """
    skip = ("suites", "_path") + (("axes",) if "suites" in run else ())

    return {k: v for k, v in run.items() if k not in skip}
