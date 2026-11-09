"""Metric screws as printed parts meet them: holes, nuts and heads, in mm.

Nominal sizes come from the ISO standards named per field. The holes are what
a printed part cuts: `clearance` for a screw passing through, `pilot` for a
screw that cuts or forms its own thread (or a tap). A heat-set insert's hole
depends on the process, so it is in `fits` (`fits.insert_hole`). A nut pocket
or a bore round a head adds a fit from `fits` to these nominals.

Values the repo's parts were already built with are pinned by
test_cadstd.py: changing one moves committed geometry.
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import StandardError


@dataclass(frozen=True)
class Screw:
    name: str
    d: float  # nominal diameter
    clearance: float  # through hole, ISO 273 medium
    pilot: float  # tap drill (ISO 2306); also the thread-forming pilot in plastic
    nut_af: float  # hex nut across flats, ISO 4032
    nut_t: float  # hex nut thickness, ISO 4032
    socket_head: tuple[float, float]  # (diameter, height), ISO 4762
    # (diameter, height), ISO 7380; None where neither ISO nor a part here gives one
    button_head: tuple[float, float] | None


M2 = Screw(
    name="M2",
    d=2.0,
    clearance=2.4,
    pilot=1.6,
    nut_af=4.0,
    nut_t=1.6,
    socket_head=(3.8, 2.0),
    # ISO 7380 starts at M3; 3.5 x 1.3 is the common vendor M2 button head
    # (the pan-tilt's head screws, hardware/pan-tilt/common/fixtures.py).
    button_head=(3.5, 1.3),
)

M2_5 = Screw(
    name="M2.5",
    d=2.5,
    clearance=2.9,
    pilot=2.05,
    nut_af=5.0,
    nut_t=2.0,
    socket_head=(4.5, 2.5),
    button_head=None,  # ISO 7380 starts at M3, and no part here uses one
)

M3 = Screw(
    name="M3",
    d=3.0,
    clearance=3.4,
    pilot=2.5,
    nut_af=5.5,
    nut_t=2.4,
    socket_head=(5.5, 3.0),
    button_head=(5.7, 1.65),
)

M4 = Screw(
    name="M4",
    d=4.0,
    clearance=4.5,
    pilot=3.3,
    nut_af=7.0,
    nut_t=3.2,
    socket_head=(7.0, 4.0),
    button_head=(7.6, 2.2),
)

SCREWS: dict[str, Screw] = {s.name: s for s in (M2, M2_5, M3, M4)}


def screw(name: str) -> Screw:
    try:
        return SCREWS[name]
    except KeyError:
        raise StandardError(
            f"no screw {name!r}; cadstd defines {sorted(SCREWS)}"
        ) from None
