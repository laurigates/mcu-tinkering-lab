"""cadstd: the shared dimensions of printed parts (ADR-030).

One source for the numbers every printed part in the repo is built from, so
parts from different projects fit together:

- `fasteners`: metric screw, nut and head sizes (ISO), with the pilot,
  clearance and heat-set insert holes printed parts use;
- `fits`: the named fits (`close`, `slide`, `running`) per print process
  (`fdm`, `sla`, `mjf`), and `MIN_WALL`;
- `grid`: the 10 mm mounting grid, its plates and slots;
- `shapes` (needs build123d): cutters and plates built from the above;
- `coupon` (needs build123d): the calibration coupon that measures a process.

The first three are stdlib only, so `tools/kicad` and `tools/hardware` may
import them; `shapes` and `coupon` import build123d and may not.
"""

from .errors import StandardError
from .fasteners import M2, M3, SCREWS, Screw, screw
from .fits import FITS, MIN_WALL, PROCESSES, fit, hole, insert_hole

__all__ = [
    "FITS",
    "M2",
    "M3",
    "MIN_WALL",
    "PROCESSES",
    "SCREWS",
    "Screw",
    "StandardError",
    "fit",
    "hole",
    "insert_hole",
    "screw",
]
