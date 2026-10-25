"""Gates for a printed mechanism: clash over its travel, and thinnest wall.

A design is a ``design.py`` exposing

- ``printed_parts() -> dict[str, Part]``: every part to order;
- ``assembly(*pose) -> list[Body]``: printed parts and stand-ins (servos,
  horns, boards, screws) in world coordinates at one pose of the joints;
- optionally ``JOINTS``: the pose's names, ``SWEEP``: the poses the clash
  gate visits, ``REST``: the pose for the excuse list and the at-rest
  renders, and ``SHOWCASE_POSES``: ``[(name, pose), ...]`` for extra renders.
  The defaults are a two-axis pan/tilt: joints (pan, tilt), pan ±90 by 30,
  tilt ±45, rest (0, 0), one view at (60, 30).

Bodies in the same ``group`` move together and may touch or overlap (a servo
in its own pocket, a horn screwed to its arm), so only pairs from different
groups are tested. ``allow`` excuses a pair by name for a deliberate interface
(a spline in its hub); every excuse is listed in the report, so a hidden clash
stays visible. A group is also the unit the exploded render lifts, so give
loose hardware its own group when it should be checked against what it
fastens.

Run: ``python -m mechanism.check <design dir>``; writes
``<design dir>/output/check.json`` and exits 1 on any clash.
"""

from __future__ import annotations

import importlib.util
import itertools
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

from build123d import Part

PAN_SWEEP = [-90, -60, -30, 0, 30, 60, 90]
TILT_SWEEP = [-45, -30, 0, 30, 45]
DEFAULT_SWEEP = [(p, t) for p in PAN_SWEEP for t in TILT_SWEEP]
DEFAULT_REST = (0, 0)
DEFAULT_JOINTS = ("pan", "tilt")
DEFAULT_POSES = [("corner-pan60-tilt30", (60, 30))]  # extra showcase views
CLASH_TOL_MM3 = 0.01


@dataclass
class Body:
    name: str
    part: Part
    group: str
    printed: bool = False
    allow: set[str] = field(default_factory=set)
    colour: str | None = None  # render colour for a printed part


def load(design_dir: Path):
    """Import ``<design_dir>/design.py`` as a module."""
    spec = importlib.util.spec_from_file_location(
        f"design_{design_dir.name}", design_dir / "design.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def poses(mod) -> list[tuple]:
    return list(getattr(mod, "SWEEP", DEFAULT_SWEEP))


def label(mod, pose: tuple) -> str:
    """``pan 60 tilt 30``: the pose with its joint names."""
    names = getattr(mod, "JOINTS", DEFAULT_JOINTS)
    return " ".join(f"{n} {v:g}" for n, v in zip(names, pose, strict=True))


def rest(mod) -> tuple:
    return tuple(getattr(mod, "REST", DEFAULT_REST))


def _apart(a: Part, b: Part) -> bool:
    p, q = a.bounding_box(), b.bounding_box()
    return (
        p.max.X < q.min.X
        or q.max.X < p.min.X
        or p.max.Y < q.min.Y
        or q.max.Y < p.min.Y
        or p.max.Z < q.min.Z
        or q.max.Z < p.min.Z
    )


def clashes(bodies: list[Body]) -> list[dict]:
    out = []
    for a, b in itertools.combinations(bodies, 2):
        if a.group == b.group or b.name in a.allow or a.name in b.allow:
            continue
        if _apart(a.part, b.part):
            continue
        common = a.part & b.part
        vol = common.volume if common is not None else 0.0
        if vol > CLASH_TOL_MM3:
            out.append({"a": a.name, "b": b.name, "mm3": round(vol, 3)})
    return out


def sweep(mod, at: list[tuple] | None = None) -> dict[str, list[dict]]:
    """Clashes at every pose, keyed by the pose."""
    results = {}
    for pose in at if at is not None else poses(mod):
        c = clashes(mod.assembly(*pose))
        if c:
            results[label(mod, pose)] = c
    return results


def excused(bodies: list[Body]) -> list[str]:
    return [f"{b.name} / {other}" for b in bodies for other in sorted(b.allow)]


def _enclosure_thickness():
    """tools/enclosure's wall gate, loaded by path: one implementation, and that
    package pins Python 3.14.8, which not every machine can install."""
    path = Path(__file__).resolve().parents[2] / "enclosure/enclosure/thickness.py"
    spec = importlib.util.spec_from_file_location("enclosure_thickness", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclass() looks its module up there
    spec.loader.exec_module(mod)
    return mod


def report(design_dir: Path) -> dict:
    thinnest = _enclosure_thickness().thinnest
    mod = load(design_dir)
    parts = {}
    for name, part in mod.printed_parts().items():
        t = thinnest(part)
        parts[name] = {
            "volume_cm3": round(part.volume / 1000, 3),
            "bbox_mm": [round(v, 1) for v in part.bounding_box().size],
            "thinnest_mm": round(t.mm, 2),
            "thinnest_at": t.where,
            "valid": part.is_valid,
        }
    return {
        "entry": design_dir.name,
        "parts": parts,
        "clashes": sweep(mod),
        "excused": excused(mod.assembly(*rest(mod))),
    }


def main(argv: list[str]) -> int:
    design_dir = Path(argv[0]).resolve()
    r = report(design_dir)
    out = design_dir / "output" / "check.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps(r, indent=2))
    return 1 if r["clashes"] else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
