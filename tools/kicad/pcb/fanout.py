"""Where a fanout via may go beside a pad: candidate centres, nearest first.

Pure geometry (stdlib `math` only), so tools/kicad/test_fanout.py tests it
without KiCad (it loads this file by path). route_board.py checks each
candidate against the routed board, in order, and takes the first that clears.
It runs under KiCad's bundled Python 3.9 as the sibling of route_board.py: no
3.10+ syntax.

A candidate sits on one of `DIRECTIONS` rays from the pad's centre, where its
centre is `gap` plus the via's radius from the pad's bounding box, then `step`
further at a time while the via's centre stays within `reach` of the box.
Distance is to the box, not along the ray: on a diagonal ray a point `gap +
radius` past the edge along the ray can sit closer than that to the edge it
crossed at a slant, or to the corner beside it (#825). They
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


def box_distance(half_w, half_h, x, y):
    """Distance from (x, y) to a 2*half_w x 2*half_h box centred on the origin; 0 inside."""
    return math.hypot(max(abs(x) - half_w, 0.0), max(abs(y) - half_h, 0.0))


def ray_point_at(half_w, half_h, dx, dy, clearance):
    """Distance along unit (dx, dy) from the box's centre to the first point
    `clearance` from the box. The distance to the box grows monotonically along
    the ray once past the edge, so step out from the edge until a point clears,
    then bisect back to the first one that does.
    """
    lo = edge_distance(half_w, half_h, dx, dy)
    hi = lo + clearance
    while box_distance(half_w, half_h, dx * hi, dy * hi) < clearance:
        hi += clearance
    for _ in range(60):
        mid = (lo + hi) / 2
        if box_distance(half_w, half_h, dx * mid, dy * mid) < clearance:
            lo = mid
        else:
            hi = mid
    return hi


def stub_width(track, half_w, half_h):
    """The stub's width: `track`, but no wider than the pad's narrow side.

    The stub starts at the pad's centre, so its round end cap is as wide as
    the stub: a 0.4 mm stub from a 0.25 mm TQFN pad at 0.5 mm pitch comes
    within 0.175 mm of the next pin before it has left its own pad, on every
    ray. No wider than the pad, it adds nothing beside the pad.
    """
    return min(track, 2 * min(half_w, half_h))


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
        n = 0
        while True:
            extra = gap + via_r + n * step
            if extra > reach + 1e-9:
                break
            d = ray_point_at(half_w, half_h, dx, dy, extra)
            turn = round(1.0 - (dx * px + dy * py), 9)
            out.append((n, turn, i, cx + dx * d, cy + dy * d))
            n += 1
    out.sort(key=lambda c: (c[0], c[1], c[2]))
    return [(x, y) for _, _, _, x, y in out]


def in_pad_points(cx, cy, orientation_deg, offsets):
    """Where `[fanout] in_pad_vias` puts its vias: each offset turned with the part.

    `offsets` are [dx, dy] in mm in the footprint's own frame (y down), so
    they follow the pad when the part turns. KiCad turns a footprint
    counter-clockwise as seen with y down: a local (x, y) lands at
    (x cos a + y sin a, -x sin a + y cos a) from the anchor. `cx`, `cy` and
    the offsets share one unit (route_board passes mm); returns (x, y) in it.
    """
    a = math.radians(orientation_deg)
    c, s = math.cos(a), math.sin(a)
    return [(cx + dx * c + dy * s, cy - dx * s + dy * c) for dx, dy in offsets]
