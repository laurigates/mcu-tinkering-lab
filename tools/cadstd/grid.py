"""The mounting grid: where reusable parts put their holes (ADR-030).

A part that mounts to, or carries, other parts puts its mounting holes on a
10 mm grid of M3 clearance holes. An odd-shaped thing (a board, a servo, a
motor) gets an adapter: a small part with its own holes on one face and grid
holes on the other. Any grid part then mounts on any other.

- Holes sit at cell centres; an n-cell plate is n * PITCH less EDGE_GAP at
  each end, so plates butted together keep every hole on one grid.
- One side of every grid joint is slotted FLOAT each way along the joint, so
  print error and a second hole pattern do not fight.
- Plates are one of PLATES thick, so M3 x 8 and M3 x 10 cover most joints.
"""

from __future__ import annotations

from .fasteners import M3, Screw

PITCH = 10.0
SCREW: Screw = M3
EDGE_GAP = 0.25  # per plate end; two butted plates leave 0.5
FLOAT = 0.5  # slot travel each way
PLATES = (3.0, 5.0)


def points(
    nx: int, ny: int, origin: tuple[float, float] = (0.0, 0.0)
) -> list[tuple[float, float]]:
    """Grid hole centres, row by row from `origin`."""
    x0, y0 = origin
    return [(x0 + i * PITCH, y0 + j * PITCH) for j in range(ny) for i in range(nx)]


def plate_size(cells: int) -> float:
    """Length of a plate `cells` grid cells long, holes at the cell centres."""
    return cells * PITCH - 2 * EDGE_GAP


def slot_length(s: Screw = SCREW) -> float:
    """End to end length of a grid slot for screw `s`."""
    return s.clearance + 2 * FLOAT


def off_grid(
    pts: list[tuple[float, float]], tol: float = 0.01
) -> list[tuple[float, float]]:
    """The points that are not a whole number of PITCHes from the first."""
    if not pts:
        return []
    x0, y0 = pts[0]

    def stray(v: float) -> bool:
        cells = v / PITCH
        return abs(cells - round(cells)) * PITCH > tol

    return [(x, y) for x, y in pts[1:] if stray(x - x0) or stray(y - y0)]
