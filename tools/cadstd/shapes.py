"""build123d cutters and plates built from the standard (ADR-035).

Every cutter stands on the XY plane and runs up +Z from z = 0 for `depth`,
centred on the origin in XY; place it with `Pos`/`Rot` and subtract it.
Sizes come from `fasteners`, `fits` and `grid`, so a part built from these
moves with the standard and with its process's measured fits.

Imports build123d: not for tools/kicad or tools/hardware.
"""

from __future__ import annotations

from build123d import (
    Align,
    Box,
    Cylinder,
    Part,
    Pos,
    RegularPolygon,
    SlotCenterToCenter,
    extrude,
)

from . import fits, grid
from .fasteners import Screw

_UP = (Align.CENTER, Align.CENTER, Align.MIN)


def bore(d: float, depth: float) -> Part:
    """A round cutter of diameter `d`."""
    return Cylinder(d / 2, depth, align=_UP)


def clearance_hole(s: Screw, depth: float) -> Part:
    return bore(s.clearance, depth)


def pilot_hole(s: Screw, depth: float) -> Part:
    return bore(s.pilot, depth)


def insert_hole(s: Screw, process: str, depth: float) -> Part:
    """Fails where `process` does not take a heat-set insert (fits.insert_hole)."""
    return bore(fits.insert_hole(s, process), depth)


def hex_prism(af: float, depth: float) -> Part:
    """A hexagonal prism `af` across flats, flats parallel to X."""
    return extrude(RegularPolygon(af / 2, 6, major_radius=False), depth)


def nut_pocket(s: Screw, depth: float, process: str, kind: str = "close") -> Part:
    """A pocket for `s`'s ISO 4032 nut, its flats `kind` fit from the nut's."""
    return hex_prism(fits.hole(s.nut_af, kind, process), depth)


def slot(s: Screw, depth: float, travel: float = grid.FLOAT) -> Part:
    """A clearance slot for `s` along X, `travel` each way of the origin."""
    return extrude(SlotCenterToCenter(2 * travel, s.clearance), depth)


def grid_plate(nx: int, ny: int, thickness: float = grid.PLATES[0]) -> Part:
    """An nx x ny cell plate with a grid clearance hole at every cell centre.

    The first hole is at the origin and the rest at `grid.points(nx, ny)`; the
    plate's underside is z = 0.
    """
    half = grid.PITCH / 2 - grid.EDGE_GAP
    plate = Pos(-half, -half, 0) * Box(
        grid.plate_size(nx),
        grid.plate_size(ny),
        thickness,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    for x, y in grid.points(nx, ny):
        plate -= Pos(x, y, -1) * clearance_hole(grid.SCREW, thickness + 2)
    return plate
