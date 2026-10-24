"""Gates every pan-tilt entry runs: clash over the travel, and thinnest wall.

An entry's ``design.py`` exposes ``assembly(pan_deg, tilt_deg)`` returning a
list of ``Body`` in world coordinates (see fixtures.py for the frame). Each
body is either a printed part (``printed=True``) or a stand-in (servo, horn,
head, screw). Bodies in the same ``group`` move together and are allowed to
touch or overlap each other (a servo in its own pocket, a horn screwed to its
arm), so only pairs from different groups are tested. A pair can be excused
with ``allow`` (names), for a deliberate interface such as a spline in its
hub; each excuse is reported.

Run: ``python -m common.check entries/<slug>`` from hardware/pan-tilt/.
"""

from __future__ import annotations

import importlib.util
import itertools
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

from build123d import Part

# Firmware boot limits (CLAUDE.md, `servo limit`): pan ±60°, tilt ±30°. The
# sweep goes past them to the servo's own travel so a widened limit is safe.
PAN_SWEEP = [-90, -60, -30, 0, 30, 60, 90]
TILT_SWEEP = [-45, -30, 0, 30, 45]
CLASH_TOL_MM3 = 0.01


@dataclass
class Body:
    name: str
    part: Part
    group: str
    printed: bool = False
    allow: set[str] = field(default_factory=set)
    colour: str | None = None  # render colour for a printed part


def load(entry: Path):
    spec = importlib.util.spec_from_file_location(
        f"entry_{entry.name}", entry / "design.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def clashes(bodies: list[Body]) -> list[dict]:
    out = []
    for a, b in itertools.combinations(bodies, 2):
        if a.group == b.group or b.name in a.allow or a.name in b.allow:
            continue
        bb_a, bb_b = a.part.bounding_box(), b.part.bounding_box()
        if (
            bb_a.max.X < bb_b.min.X
            or bb_b.max.X < bb_a.min.X
            or bb_a.max.Y < bb_b.min.Y
            or bb_b.max.Y < bb_a.min.Y
            or bb_a.max.Z < bb_b.min.Z
            or bb_b.max.Z < bb_a.min.Z
        ):
            continue
        common = a.part & b.part
        vol = common.volume if common is not None else 0.0
        if vol > CLASH_TOL_MM3:
            out.append({"a": a.name, "b": b.name, "mm3": round(vol, 3)})
    return out


def sweep(mod, pans=PAN_SWEEP, tilts=TILT_SWEEP) -> dict:
    """Clash at every pose; the travel each axis clears with the other at 0."""
    results = {}
    for p in pans:
        for t in tilts:
            c = clashes(mod.assembly(p, t))
            if c:
                results[f"pan {p} tilt {t}"] = c
    return results


def _enclosure_thickness():
    """tools/enclosure's wall gate, loaded by path: one implementation, and the
    package itself pins Python 3.14.8, which not every machine has."""
    path = (
        Path(__file__).resolve().parents[6] / "tools/enclosure/enclosure/thickness.py"
    )
    spec = importlib.util.spec_from_file_location("enclosure_thickness", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclass() looks its module up there
    spec.loader.exec_module(mod)
    return mod


def main(argv: list[str]) -> int:
    thinnest = _enclosure_thickness().thinnest

    entry = Path(argv[0]).resolve()
    mod = load(entry)
    report = {"entry": entry.name, "parts": {}, "clashes": {}, "excused": []}
    for name, part in mod.printed_parts().items():
        t = thinnest(part)
        report["parts"][name] = {
            "volume_cm3": round(part.volume / 1000, 3),
            "bbox_mm": [round(v, 1) for v in part.bounding_box().size],
            "thinnest_mm": round(t.mm, 2),
            "thinnest_at": t.where,
            "valid": part.is_valid,
        }
    for b in mod.assembly(0, 0):
        for other in sorted(b.allow):
            report["excused"].append(f"{b.name} / {other}")
    report["clashes"] = sweep(mod)
    out = entry / "output" / "check.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 1 if report["clashes"] else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
