"""Named fits per print process, and the thinnest wall any part may have.

A fit is the gap between two faces, per side. A bore for a pin of diameter d
is `hole(d, kind, process)` = d + 2 * gap; a slot for a tab of width w is the
same call. Parts ask for a fit by name, never for a number, so measuring a
process (the calibration coupon, `just cadstd::coupon`) changes every part
built for it at once.

The three classes:

- `close`: located and then clamped, so it does not move once tightened: a
  captive nut in its pocket, a split collar on its rod, a part on a dowel.
- `slide`: assembles by hand and stays put under its own weight: a lid lip in
  its wall, a key in its slot.
- `running`: moves against its neighbour in use: a yoke beside the head it
  carries, a cheek beside a turning horn.

No `press` class: nothing in the repo relies on an interference fit yet, and
none has been measured. The coupon's hole ladder starts at 0.05 per side, so
one can be measured before it is added.

Until a process is measured, its values are the ones the repo's parts were
built with, and each says so in `source`.
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import StandardError
from .fasteners import Screw

# Thinnest wall any feature may have. JLC3DP's upload page asks for walls over
# 1.2 mm, but its quote still flagged "Thin walls detected" on lids whose
# thinnest wall measured 1.6 mm, so the margin is wider than the page says.
# Four 0.45 mm FDM perimeters also fit in it.
MIN_WALL = 2.0

FITS = ("close", "slide", "running")


@dataclass(frozen=True)
class Fit:
    gap: float  # per side, mm
    source: str  # where the value came from: a design, or a measured coupon
    measured: bool = False


@dataclass(frozen=True)
class Insert:
    hole: float  # mm
    source: str
    measured: bool = False


@dataclass(frozen=True)
class Process:
    name: str
    description: str
    fits: dict[str, Fit]
    # Heat-set insert holes by screw name. Empty: the process takes none.
    inserts: dict[str, Insert]


_CLOSE = Fit(0.15, "pan-tilt M3 nut pocket (5.8 AF), microscope rod bore (10.3)")
_SLIDE = Fit(0.3, "case.py LIP_FIT (resin), pan-tilt KEY_FIT (PA12)")
_RUNNING = Fit(0.5, "pan-tilt GAP, parts that move against each other (PA12)")

PROCESSES: dict[str, Process] = {
    p.name: p
    for p in (
        Process(
            name="fdm",
            description="FDM, 0.4 mm nozzle, PLA or PETG (home printer)",
            fits={"close": _CLOSE, "slide": _SLIDE, "running": _RUNNING},
            inserts={
                "M3": Insert(
                    4.0, "estimate: common M3 x 5.7 insert sheets; not printed"
                ),
            },
        ),
        Process(
            name="sla",
            description="JLC3DP resin (SLA): the carrier cases",
            fits={"close": _CLOSE, "slide": _SLIDE, "running": _RUNNING},
            # Resin takes a thread well but cracks round a hot insert; JLC
            # press-fits or bonds them instead (docs/reference/jlc-blog,
            # 3dp-enclosure-mechanism.md C2). None chosen here yet.
            inserts={},
        ),
        Process(
            name="mjf",
            description="JLC3DP PA12 (MJF): the pan-tilt",
            fits={"close": _CLOSE, "slide": _SLIDE, "running": _RUNNING},
            inserts={
                "M2": Insert(3.2, "pan-tilt print: M2 x 3 insert, OD 3.5 knurl"),
                "M3": Insert(4.2, "JLC3DP threaded-insert post: M3 in MJF 4.2-4.3"),
            },
        ),
    )
}


def process(name: str) -> Process:
    try:
        return PROCESSES[name]
    except KeyError:
        raise StandardError(
            f"no process {name!r}; cadstd defines {sorted(PROCESSES)}"
        ) from None


def fit(kind: str, process_name: str) -> float:
    """The per-side gap of fit `kind` on `process_name`."""
    p = process(process_name)
    if kind not in p.fits:
        raise StandardError(f"no fit {kind!r}; cadstd defines {list(FITS)}")
    return p.fits[kind].gap


def hole(nominal: float, kind: str, process_name: str) -> float:
    """A bore or slot width that takes `nominal` with fit `kind`."""
    return nominal + 2 * fit(kind, process_name)


def insert_hole(s: Screw, process_name: str) -> float:
    """The heat-set insert hole for `s` on `process_name`; fails where the
    process or the size has no insert, rather than cutting a hole nothing
    will fill."""
    p = process(process_name)
    if not p.inserts:
        raise StandardError(
            f"{p.name} ({p.description}) does not take heat-set inserts; "
            f"use a pilot ({s.pilot} mm) or a nut pocket"
        )
    if s.name not in p.inserts:
        raise StandardError(
            f"no {s.name} insert on {p.name}; cadstd has {sorted(p.inserts)}"
        )
    return p.inserts[s.name].hole
