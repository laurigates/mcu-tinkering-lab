"""Stitching-via candidates for a poured net (pcbnew-free).

board.toml ``[pour] stitch = { pitch_mm, via_um, exclude }`` asks
route_board.py for a via of the pour's net at each point of a grid where the
fill covers the whole via, with its clearance, on every pour layer. Freerouting
adds vias only where a track changes layer and ``[fanout]`` only beside SMD
pads, so a pour region with neither (the space between a socketed module's
pin rows) otherwise reaches the other layer through a thermal spoke of one
pad. This module only lays out the grid; route_board.py tests each point
against the fill and adds the via.

Loaded by path from route_board.py and test_stitch.py (`unittest discover`
does not reach pcb/), so it imports nothing from KiCad.
"""

import math


def grid(width, height, pitch, exclude=()):
    """Points (x, y) in mm, origin-relative, `pitch` apart over a width x height
    outline, centred so the margins left at opposite edges are equal, without
    the points inside any `exclude` rectangle [x0, y0, x1, y1] (edges included).
    """
    if pitch <= 0:
        raise ValueError(f"stitch pitch must be > 0, got {pitch}")
    nx = int(width // pitch)
    ny = int(height // pitch)
    x0 = (width - (nx - 1) * pitch) / 2 if nx else width / 2
    y0 = (height - (ny - 1) * pitch) / 2 if ny else height / 2
    points = []
    for j in range(max(ny, 1)):
        for i in range(max(nx, 1)):
            x, y = round(x0 + i * pitch, 4), round(y0 + j * pitch, 4)
            if any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in exclude):
                continue
            points.append((x, y))
    return points


def ring(x, y, radius, n=16):
    """`n` points on a circle around (x, y), and the centre: what must be in the fill."""
    return [(x, y)] + [
        (
            x + radius * math.cos(2 * math.pi * k / n),
            y + radius * math.sin(2 * math.pi * k / n),
        )
        for k in range(n)
    ]
