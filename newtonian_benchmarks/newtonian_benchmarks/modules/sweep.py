
from __future__ import annotations

from typing import Any

import hashlib
import itertools
import json
import random
from pathlib import Path

import numpy as np
import yaml

from newtonian_benchmarks.modules.config import defaults


COMMAND_KEYS = ("vx", "vy", "wz", "height", "profile", "ramp", "freq", "go", "duration", "settle")
OTHER_KEYS = ("friction", "payload", "payload_frac", "cmd_latency", "sensor_latency", "jitter", "drop", "push", "terrain", "start", "rtf")
KEYS = COMMAND_KEYS + OTHER_KEYS

# keys of a suite that are merged, key by key, over the defaults of the file
MERGE_KEYS = ("trial", "thresholds", "pass", "fixed", "sim")


def label(value: Any) -> str:
    """
    Returns a short text for an axis value.
    """
    if isinstance(value, dict):
        return "_".join(str(v) for v in value.values())

    return str(value)


def expand(suite: dict[str, Any], limits: dict[str, list[float]] | None = None) -> list[dict[str, Any]]:
    """
    Returns the trials of 'suite'.
    """
    limits = limits or {}
    fixed = dict(suite.get("fixed", {}))

    # values of each axis
    axes: dict[str, list[Any]] = {}
    for name, values in suite.get("axes", {}).items():
        if name not in KEYS:
            raise ValueError(f"unknown axis {name!r}; allowed {KEYS}")
        axes[name] = _values(values=values, limits=limits)

    for name in fixed:
        if name not in KEYS:
            raise ValueError(f"unknown key {name!r}; allowed {KEYS}")

    # points of the sweep
    combos = _points(axes=axes, suite=suite)

    # one trial for each point and seed
    trials = []
    for combo in combos:
        for seed in suite.get("seeds", [0]):
            varied = dict(zip(axes, combo))
            params = {**fixed, **varied}

            # the suite name is part of the id in a file with several suites, so that two suites can hold the same point
            key = [suite.get("name"), params, seed] if suite.get("_in_run") else [params, seed]
            tid = hashlib.sha1(json.dumps(key, sort_keys=True).encode()).hexdigest()[:10]

            trials.append({"id": tid, "axes": {k: label(value=v) for k, v in varied.items()}, "params": params, "seed": seed})

    return trials


def _values(values: Any, limits: dict[str, list[float]]) -> list[Any]:
    """
    Returns the values of one axis.
    """
    if isinstance(values, dict) and "frac_of" in values:
        upper = limits[values["frac_of"]][1]
        return [round(float(v) * upper, 4) for v in values["values"]]

    if isinstance(values, dict) and "from_limits" in values:
        lo, hi = limits[values["from_limits"]]
        n = int(values.get("n", 5))
        k = float(values.get("scale", 1.0))
        return [round(lo * k + (hi * k - lo * k) * i / max(n - 1, 1), 4) for i in range(n)]

    return list(values)


def _points(axes: dict[str, list[Any]], suite: dict[str, Any]) -> list[tuple]:
    """
    Returns the points of the sweep.
    """
    sampling = suite.get("sampling", "grid")
    names = list(axes)

    # all combinations
    if sampling == "grid":
        return list(itertools.product(*axes.values())) if axes else [()]

    # the first value of each axis, then each axis alone with its other values
    if sampling == "ablation":
        base = [axes[n][0] for n in names]
        combos = [tuple(base)]
        for i, n in enumerate(names):
            for v in axes[n][1:]:
                c = list(base)
                c[i] = v
                combos.append(tuple(c))
        return combos

    if sampling != "random":
        raise ValueError(f"unknown sampling {sampling!r}")

    # points of the full product, without repeats ; all of them if 'samples' is not smaller than the product
    rng = random.Random(int(suite.get("sampling_seed", 0)))
    sizes = [len(axes[n]) for n in names]
    total = int(np.prod(sizes)) if sizes else 1
    picks = rng.sample(range(total), min(int(suite.get("samples", 20)), total))

    combos = []
    for index in sorted(picks):

        # index in the product to one value of each axis (the last axis changes fastest)
        combo = []
        for n, size in zip(reversed(names), reversed(sizes)):
            index, k = divmod(index, size)
            combo.append(axes[n][k])
        combos.append(tuple(reversed(combo)))

    return combos


def find_suites(run: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Returns the suites of 'run'.
    """
    base = defaults(run=run)
    if "suites" not in run:
        return [{**base, "name": base.get("name", "suite")}]

    library = run.get("axes", {})
    out = []
    for name, over in run["suites"].items():
        over = over or {}
        suite: dict[str, Any] = {**base, "name": name, "_in_run": True}

        # keys of the suite over the defaults
        for k, v in over.items():
            if k in ("cross", "axes"):
                continue
            suite[k] = {**base.get(k, {}), **v} if k in MERGE_KEYS and isinstance(v, dict) else v

        # axes : the crossed ones of the library, then the own ones
        axes: dict[str, Any] = {}
        for item in over.get("cross", []):
            if isinstance(item, str):
                if item not in library:
                    raise ValueError(f"suite {name!r}: axis {item!r} is not in the 'axes' library {sorted(library)}")
                axes[item] = library[item]
            else:
                axes.update(item)
        axes.update(over.get("axes", {}))

        suite["axes"] = axes
        out.append(suite)

    return out


def expand_run(run: dict[str, Any], limits: dict[str, list[float]] | None = None, only: list[str] | None = None) -> list[dict[str, Any]]:
    """
    Returns the trials of the suites of 'run'.
    """
    suites = find_suites(run=run)

    if only:
        unknown = sorted(set(only) - {s["name"] for s in suites})
        if unknown:
            raise ValueError(f"unknown suites {unknown}; the file has {[s['name'] for s in suites]}")
        suites = [s for s in suites if s["name"] in only]

    trials = []
    for suite in suites:
        cfg = {k: v for k, v in suite.items() if k != "axes"}
        for t in expand(suite=suite, limits=limits):
            trials.append({**t, "suite": suite["name"], "cfg": cfg})

    return trials


def scene_key(params: dict[str, Any], suite: dict[str, Any]) -> str:
    """
    Returns the text that identifies the scene of a trial.
    """
    return json.dumps({"terrain": params.get("terrain", "flat") or "flat", "sim": suite.get("sim", {})}, sort_keys=True)


def write_scene(base: dict[str, Any], key: str, out: Path) -> Path:
    """
    Writes the scene file of 'key' to 'out' and returns its path.
    """
    spec = json.loads(key)
    cfg = json.loads(json.dumps(base))
    rp = cfg["newtonian"]["ros__parameters"]

    # terrain : one more environment, with the friction of the ground
    terrain = spec["terrain"]
    if terrain not in ("flat", None):
        ground = rp["environments"].get("ground", {})
        rp.setdefault("environments", {})["terrain"] = {"type": "terrain", "mu": ground.get("mu", 1.0), **({"color": list(ground["color"])} if "color" in ground else {}), **terrain}

        # a terrain that goes down replaces the ground plane, which would hide it
        if terrain.get("direction") == "down":
            rp["environments"] = {k: v for k, v in rp["environments"].items() if v.get("type") != "groundplane"}

    # values of 'sim' of the suite
    for k, v in spec["sim"].items():
        rp.setdefault("sim", {})[k] = v

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(cfg, sort_keys=False))

    return out
