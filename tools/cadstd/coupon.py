"""The calibration coupon: measure a process's fits before trusting them.

Print a plate and its peg on the process, then:

1. Push the peg into each ladder hole (front row, per-side gaps 0.05 to 0.5,
   smallest at the chamfered corner). Read each class off the smallest gap
   that does what the class needs: the peg enters with a light tap and does
   not wobble (`close`); it slides in and out by hand (`slide`); it turns
   freely with no drag (`running`).
2. Try the hardware in the back row: an M3 screw through the clearance hole
   and the slot, M3 and M2 screws into their pilots, an M3 nut in its pocket
   from the top, and (where the process takes them) the heat-set inserts.
3. Lay a grid plate (or a ruler) over the back row: its holes are on the
   10 mm pitch, so a short or long print shows here.

Write what you measured into `fits.PROCESSES` with `measured=True` and the
date in `source`; every part built for that process follows.

    python -m cadstd.coupon <process> [out dir]   # just cadstd::coupon <process>

Imports build123d: not for tools/kicad or tools/hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import Align, Box, Cylinder, Part, Pos, Rot, export_step, export_stl

from . import fits, grid, shapes
from .fasteners import M2, M3

PEG_D = 4.0  # the widest ladder hole (5.0) still leaves MIN_WALL to the edge
LADDER = (0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5)  # per-side gaps, front row
THICKNESS = grid.PLATES[1]
CORNER = 2.0  # chamfer legs at the smallest-gap corner, so the order reads
# Behind the back row: the M3 nut pocket's flats sit 2.9 from its centre, so
# a grid-sized plate leaves 1.85 mm there, under MIN_WALL.
BACK_MARGIN = 1.0


def plate(process: str) -> Part:
    """The coupon plate: the hole ladder in front (y 0), hardware behind (y 10)."""
    half = grid.PITCH / 2 - grid.EDGE_GAP
    body = Pos(-half, -half, 0) * Box(
        grid.plate_size(len(LADDER)),
        grid.plate_size(2) + BACK_MARGIN,
        THICKNESS,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    through = THICKNESS + 2
    for (x, _), gap in zip(grid.points(len(LADDER), 1), LADDER, strict=True):
        body -= Pos(x, 0, -1) * shapes.bore(PEG_D + 2 * gap, through)
    # A rotated square on the corner cuts a chamfer with legs CORNER.
    body -= (
        Pos(-half, -half, -1)
        * Rot(0, 0, 45)
        * Box(
            CORNER * 2**0.5,
            CORNER * 2**0.5,
            through,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )

    for x, cutter in enumerate(_back_row(process)):
        body -= Pos(x * grid.PITCH, grid.PITCH, 0) * cutter
    return body


def _back_row(process: str) -> list[Part]:
    """The hardware cutters, one per grid cell along the back row."""
    through = THICKNESS + 2
    pilot_depth = THICKNESS - fits.MIN_WALL + 1  # blind: a skin stays under it
    nut_z = THICKNESS - M3.nut_t
    row = [
        Pos(0, 0, -1) * shapes.clearance_hole(M3, through),
        Pos(0, 0, -1) * shapes.slot(M3, through),
        Pos(0, 0, fits.MIN_WALL) * shapes.pilot_hole(M3, pilot_depth),
        Pos(0, 0, fits.MIN_WALL) * shapes.pilot_hole(M2, pilot_depth),
        # The nut drops in from the top; its screw comes up through the plate.
        Pos(0, 0, nut_z) * shapes.nut_pocket(M3, M3.nut_t + 1, process)
        + Pos(0, 0, -1) * shapes.clearance_hole(M3, through),
    ]
    # Through, so a long insert still seats in the 5 mm plate.
    inserts = fits.process(process).inserts
    row += [
        Pos(0, 0, -1) * shapes.insert_hole(s, process, through)
        for s in (M2, M3)
        if s.name in inserts
    ]
    if len(row) > len(LADDER):
        raise ValueError(f"back row has {len(row)} cells, the plate {len(LADDER)}")
    return row


def peg() -> Part:
    """The reference peg: printed on the same process, so the ladder measures
    print against print, the fit printed assemblies have."""
    return Cylinder(
        PEG_D / 2, THICKNESS + 5, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def coupon(process: str) -> dict[str, Part]:
    return {"plate": plate(process), "peg": peg()}


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    process = argv[0]
    out = Path(argv[1]) if len(argv) > 1 else Path(__file__).parent / "output"
    out.mkdir(parents=True, exist_ok=True)
    for name, part in coupon(process).items():
        stem = out / f"coupon-{process}-{name}"
        export_stl(part, f"{stem}.stl")
        export_step(part, f"{stem}.step")
        print(f"{stem}.stl")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
