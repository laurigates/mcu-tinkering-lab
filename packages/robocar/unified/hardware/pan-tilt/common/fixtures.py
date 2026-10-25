"""Stand-in solids every pan-tilt entry is checked against.

Shared by all entries so the clash and fit checks compare like with like. The
servo envelopes are tools/mechanism's (``mechanism.servos``, re-exported here
so entries keep writing ``fx.servo()``); this module adds robocar-unified's
head 1. Every solid is a keep-out envelope at least as large as the part, and
every dimension names its source; "estimate" means nobody has put calipers on
it.

World frame (all entries): +X forward (the camera looks along +X), +Y left,
+Z up, millimetres.
"""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Location, Part, Pos
from mechanism.servos import (  # noqa: F401 (re-exported for entries)
    CENTER_BOTTOM,
    CENTER_TOP,
    MARGIN,
    MG90D,
    SG90,
    ServoSpec,
    horn,
    servo,
    servo_tab_holes,
)

# --- Head board (robocar-unified head 1) ------------------------------------
# hardware/carrier/board.toml: a 29.5 x 29.5 mm square, 1.6 mm FR-4; M2 holes
# (2.2 mm) at 4.0 and 26.5 mm from the left edge, 3.05 mm below the top edge;
# the XIAO ESP32-S3 Sense centred 15.25 mm from the left, 12.7 mm below the top,
# USB-C toward the top edge; J2 (2x7 2.0 mm header, back face) centred 21.25 mm
# from the left, 3.5 mm above the bottom edge.

HEAD_SIZE = 29.5
HEAD_THICK = 1.6
HEAD_HOLES_YZ = [(-11.25, HEAD_SIZE - 3.05), (11.25, HEAD_SIZE - 3.05)]  # (y, z)
# enclosure.toml U1: 26 mm above the board top with sockets and the camera board.
XIAO_STACK = (26.0, 17.8, 21.0)  # (x depth forward, y width, z height); Seeed 21 x 17.8
XIAO_CENTRE_YZ = (0.5, HEAD_SIZE - 12.7)
# J2 header 14 x 4 mm, ~6 mm pins, plus the IDC plug (6 mm) and the ribbon's
# bend radius: estimate 16 mm behind the board, 18 x 8 mm.
J2_CENTRE_YZ = (-6.5, 3.5)
J2_KEEPOUT = (16.0, 18.0, 8.0)  # (x depth back, y, z)


def head(screws: bool = True) -> Part:
    """Head 1 populated, board plane YZ at x 0..1.6, front face +X.

    Board bottom edge at z 0, centred on y 0. The camera looks along +X from
    the front of the XIAO stack, lens about the stack's centre (estimate).
    """
    board = Pos(0, 0, 0) * Box(
        HEAD_THICK, HEAD_SIZE, HEAD_SIZE, align=(Align.MIN, Align.CENTER, Align.MIN)
    )
    xs, ys, zs = XIAO_STACK
    stack = Pos(HEAD_THICK, XIAO_CENTRE_YZ[0], XIAO_CENTRE_YZ[1]) * Box(
        xs, ys, zs, align=(Align.MIN, Align.CENTER, Align.CENTER)
    )
    xd, yd, zd = J2_KEEPOUT
    j2 = Pos(0, J2_CENTRE_YZ[0], J2_CENTRE_YZ[1]) * Box(
        xd, yd, zd, align=(Align.MAX, Align.CENTER, Align.CENTER)
    )
    env = board + stack + j2
    if screws:
        # M2 button heads (ISO 7380: Ø3.5 x 1.3) on the front face.
        for y, z in HEAD_HOLES_YZ:
            env += Location((HEAD_THICK, y, z), (0, 90, 0)) * Cylinder(
                1.75, 1.3, align=CENTER_BOTTOM
            )
    return env


CAMERA_LENS_FROM_BOARD = XIAO_STACK[0]  # lens plane, x from the board's back face + 1.6
