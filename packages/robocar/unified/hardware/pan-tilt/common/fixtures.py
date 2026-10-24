"""Stand-in solids every pan-tilt entry is checked against.

Shared by all entries so the clash and fit checks compare like with like. They
are keep-out envelopes, not models: each solid is at least as large as the part
it stands for, and every dimension names its source. "estimate" means nobody
has put calipers on it; measure the parts in hand before ordering a print.

World frame (all entries): +X forward (the camera looks along +X), +Y left,
+Z up, millimetres.

Servo frame (``servo()``): the output shaft's axis is the Z axis, z 0 is the
top of the case (not the spline), the body hangs below in -Z, and the case's
long axis is X with the shaft near its +X end.
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Location, Part, Pos

CENTER_BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)
CENTER_TOP = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class ServoSpec:
    name: str
    length: float  # B: case length, along X
    width: float  # D: case width, along Y
    height: float  # C: base to case top, excluding the spline
    overall: float  # A: base to spline top
    tab_span: float  # E: across the mounting tabs, along X
    tab_underside: float  # F: base to the underside of the tabs
    tab_thickness: float
    tab_hole_inset: float  # tab hole centre from the tab's outer end
    tab_hole_d: float
    shaft_from_end: float  # shaft axis from the case's +X end
    spline_d: float
    source: str


# TowerPro product page, read 2026-09-15 (docs/reference/datasheets/actuator--mg90d.md):
# A 32.5, B 22.6, C 28.5, D 12, E 31.5, F 19.8. The rest are estimates.
# Lauri (2026-10-24): the MG90D is taller than the SG90 and does not fit the
# generic black 9 g kits. F is the reason on paper: 19.8 mm below the tabs
# against the SG90's ~16, so it bottoms out in the kit's pan cup.
MG90D = ServoSpec(
    name="MG90D",
    length=22.6,
    width=12.0,
    height=28.5,
    overall=32.5,
    tab_span=31.5,
    tab_underside=19.8,
    tab_thickness=2.5,  # estimate
    tab_hole_inset=2.3,  # estimate
    tab_hole_d=2.0,  # estimate
    shaft_from_end=6.0,  # estimate
    spline_d=4.8,  # estimate (20T micro spline)
    source="TowerPro MG90D page (A-F); tab thickness, holes, shaft offset estimates",
)

# TowerPro SG90 page: 23 x 12.2 x 29, E 32.3. A, F and the rest are estimates
# from the common SG90 drawing (Handsontec).
SG90 = ServoSpec(
    name="SG90",
    length=23.0,
    width=12.2,
    height=29.0,
    overall=32.0,  # estimate
    tab_span=32.3,
    tab_underside=15.9,  # estimate
    tab_thickness=2.5,  # estimate
    tab_hole_inset=2.3,  # estimate
    tab_hole_d=2.0,  # estimate
    shaft_from_end=6.0,  # estimate
    spline_d=4.8,  # estimate
    source="TowerPro SG90 page (B, C, D, E); A, F and the rest from the common drawing, estimates",
)

# Added to every envelope dimension: tolerance between a real servo and the
# published figure, plus print tolerance. Not a fit clearance for the cavity;
# the entry chooses its own.
MARGIN = 0.0


def servo(spec: ServoSpec = MG90D, margin: float = MARGIN, cable: bool = True) -> Part:
    """The servo's keep-out envelope in the servo frame (see the module doc)."""
    m = margin
    # body spans x from shaft_from_end - length to shaft_from_end
    body = Pos(spec.shaft_from_end - spec.length / 2, 0, m) * Box(
        spec.length + 2 * m, spec.width + 2 * m, spec.height + 2 * m, align=CENTER_TOP
    )
    tab_z_bottom = -(spec.height - spec.tab_underside)
    tabs = Pos(spec.shaft_from_end - spec.length / 2, 0, tab_z_bottom - m) * Box(
        spec.tab_span + 2 * m,
        spec.width + 2 * m,
        spec.tab_thickness + 2 * m,
        align=CENTER_BOTTOM,
    )
    spline = Cylinder(
        spec.spline_d / 2 + m, spec.overall - spec.height + m, align=CENTER_BOTTOM
    )
    env = body + tabs + spline
    if cable:
        # Lead exits the -X end face low on the case, bends down or sideways.
        # estimate: 4 x 3 mm boot, 6 mm of lead before it can bend.
        boot = Pos(
            spec.shaft_from_end - spec.length - 3.0, 0, -spec.height + 5.0
        ) * Box(6.0 + 2 * m, 4.0 + 2 * m, 4.0 + 2 * m, align=CENTER_BOTTOM)
        env = env + boot
    return env


def servo_tab_holes(spec: ServoSpec = MG90D) -> list[tuple[float, float, float]]:
    """Tab hole centres (servo frame) at the tabs' underside."""
    cx = spec.shaft_from_end - spec.length / 2
    half = spec.tab_span / 2 - spec.tab_hole_inset
    z = -(spec.height - spec.tab_underside)
    return [(cx - half, 0.0, z), (cx + half, 0.0, z)]


def horn(kind: str = "single", spec: ServoSpec = MG90D) -> Part:
    """Stock horn envelope on the spline, in the servo frame.

    Hub Ø7 x 4 from 0.5 mm above the case; arm 1.6 mm thick at the hub's top.
    single: one arm 20 mm from the axis along +X. double: both directions.
    cross: four arms 16 mm. disc: Ø21. All estimates of the bag of horns
    shipped with micro servos.
    """
    z0 = 0.5
    hub = Pos(0, 0, z0) * Cylinder(3.5, 4.0, align=CENTER_BOTTOM)
    arm_z = z0 + 4.0 - 1.6
    if kind == "disc":
        return hub + Pos(0, 0, arm_z) * Cylinder(10.5, 1.6, align=CENTER_BOTTOM)
    if kind == "single":
        return hub + Pos(10, 0, arm_z) * Box(20, 5.5, 1.6, align=CENTER_BOTTOM)
    if kind == "double":
        return hub + Pos(0, 0, arm_z) * Box(40, 5.5, 1.6, align=CENTER_BOTTOM)
    if kind == "cross":
        a = Pos(0, 0, arm_z) * Box(32, 5.0, 1.6, align=CENTER_BOTTOM)
        b = Pos(0, 0, arm_z) * Box(5.0, 32, 1.6, align=CENTER_BOTTOM)
        return hub + a + b
    raise ValueError(kind)


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
