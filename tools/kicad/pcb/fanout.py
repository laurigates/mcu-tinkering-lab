"""Where a fanout via may go beside a pad: candidate centres, nearest first.

Pure geometry (stdlib `math` only), so tools/kicad/test_fanout.py tests it
without KiCad (it loads this file by path). route_board.py checks each
candidate against the routed board, in order, and takes the first that clears.
It runs under KiCad's bundled Python 3.9 as the sibling of route_board.py: no
3.10+ syntax.

A candidate sits on one of `DIRECTIONS` rays from the pad's centre, `gap` plus
the via's radius beyond the edge of the pad's bounding box, then `step` further
out at a time while the via's centre stays within `reach` of that edge. They
are ordered by distance beyond the edge, then by how far the ray turns from
`prefer` (a direction, normally away from the pad's own part, where a via
blocks nothing else's way out), then by ray (0 is +x, counting toward +y), so
the result does not depend on anything but the arguments.
"""

import math

DIRECTIONS = 16


def edge_distance(half_w, half_h, dx, dy):
    """Distance from the centre of a 2*half_w x 2*half_h box to its edge along unit (dx, dy)."""
    tx = half_w / abs(dx) if dx else math.inf
    ty = half_h / abs(dy) if dy else math.inf
    return min(tx, ty)


def candidates(cx, cy, half_w, half_h, via_r, gap, reach, step, prefer=(0.0, 0.0)):
    """(x, y) via centres around the box centred on (cx, cy), nearest first."""
    px, py = prefer
    pn = math.hypot(px, py)
    px, py = (px / pn, py / pn) if pn else (0.0, 0.0)
    out = []
    for i in range(DIRECTIONS):
        a = 2 * math.pi * i / DIRECTIONS
        dx, dy = math.cos(a), math.sin(a)
        dx = 0.0 if abs(dx) < 1e-12 else dx
        dy = 0.0 if abs(dy) < 1e-12 else dy
        edge = edge_distance(half_w, half_h, dx, dy)
        n = 0
        while True:
            extra = gap + via_r + n * step
            if extra > reach + 1e-9:
                break
            d = edge + extra
            turn = round(1.0 - (dx * px + dy * py), 9)
            out.append((n, turn, i, cx + dx * d, cy + dy * d))
            n += 1
    out.sort(key=lambda c: (c[0], c[1], c[2]))
    return [(x, y) for _, _, _, x, y in out]
