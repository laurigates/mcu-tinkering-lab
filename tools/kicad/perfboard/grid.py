"""The hole grid: pitch, rotation, snapping millimetres to holes, hole names.

Holes are (column, row), 0-based, named as seen from the component side:
columns A..Z, then AA, AB, ... as a spreadsheet names them; rows from 1.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..errors import KicadError

PITCH = 2.54


@dataclass(frozen=True)
class Board:
    cols: int
    rows: int


def rotate(x, y, deg):
    """KiCad footprint rotation, y down: positive is counter-clockwise on screen."""
    r = {0: (x, y), 90: (y, -x), 180: (-x, -y), 270: (-y, x)}
    return r[deg % 360]


def to_hole(mm_value, what):
    h = mm_value / PITCH
    if abs(h - round(h)) > 0.1:  # 2.50 mm leads bend onto a 2.54 mm grid
        raise KicadError(f"{what}: {mm_value} mm is not on the 2.54 mm grid")
    return round(h)


def col_name(c):
    """0 -> "A", 25 -> "Z", 26 -> "AA", 27 -> "AB"; a column left of the board,
    which only an off-the-board error names, is "[-1]"."""
    if c < 0:
        return f"[{c}]"
    name = ""
    c += 1
    while c:
        c, rem = divmod(c - 1, 26)
        name = chr(ord("A") + rem) + name
    return name


def hole_name(c, r):
    return f"{col_name(c)}{r + 1}"


def overlaps(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
