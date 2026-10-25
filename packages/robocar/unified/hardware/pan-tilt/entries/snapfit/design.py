"""Snap & Flex: a two-MG90D pan-tilt for robocar-unified head 1 that clicks
together. Two M3 screws hold it to the chassis; nothing else needs a tool.

    base    drops on the chassis (2 x M3). The pan servo DROPS IN from the top,
            tabs resting on the pocket walls; nothing screws it down.
    yoke    the cross horn presses into its underside pocket, then the whole yoke
            presses down on the base: four cantilever fingers click under the
            base ring's lip. That snap is a captured pivot: the ring takes the
            thrust and the tipping load, holds the horn on the spline and the pan
            servo in its pocket, so neither needs a screw.
            The tilt servo SLIDES IN from the front along two tab grooves until a
            flexure finger in the lower groove clicks behind its tab.
    cradle  the head board DROPS IN from the top along two edge grooves; two
            latch tongues behind it snap printed pegs into its M2 holes. The
            single-arm horn (screwed to the tilt spline first, or not) presses
            into a pocket in the cradle's -Y cheek.
            Its +Y cheek carries a Ø6 pivot stub that the yoke's +Y arm springs
            over: the opposite-side pivot, so the tilt shaft is not a
            cantilever, and the stop that keeps the horn on its spline.
    brows   (EYEBROWS = True) the cradle grows a deck behind the board's top
            edge carrying two SG90 slide cradles (same groove + flexure finger);
            each SG90 slides in sideways and clicks, and a bushy printed brow
            presses onto each single-arm horn. Teuvo frowns.

Frames: world per common/fixtures.py (+X forward, +Y left, +Z up, mm), the pan
axis at x = y = 0 and the chassis plate's top at z = 0. ``printed_parts()``
returns every part in its WORLD pose at pan 0, tilt 0 (print orientation is in
the README: MJF needs no supports, so orientation only matters for finish and
for the direction the snap beams' layers run).

Run ``python design.py`` to write output/<part>.step and .stl.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from build123d import (
    Align,
    Box,
    Cone,
    Cylinder,
    Location,
    Part,
    Plane,
    Polygon,
    Pos,
    Rot,
    export_step,
    export_stl,
    extrude,
    mirror,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))  # hardware/pan-tilt, for common.*

from common import fixtures as fx  # noqa: E402
from common.check import Body  # noqa: E402

MIN, CEN, MAX = Align.MIN, Align.CENTER, Align.MAX

# ---------------------------------------------------------------------------
# CALIPER THESE before ordering (see README "Measure first"). Every snap below
# is sized from them; a wrong tab thickness is the one thing that stops a
# slide-in groove from working.
# ---------------------------------------------------------------------------
SERVO = fx.MG90D  # tab span E 31.5, below-tab F 19.8, case 22.6 x 12 x 28.5
TAB_T = SERVO.tab_thickness  # 2.5 estimate: the grooves are TAB_T + 2 * TAB_CLR
SHAFT_FROM_END = SERVO.shaft_from_end  # 6.0 estimate
EYEBROWS = True
BROW = fx.SG90

# ---------------------------------------------------------------------------
# Fit (JLC3DP MJF: +-0.3 mm holes, 0.2-0.4 mm assembly clearance, 0.6 mm for
# anything that must not fuse inside one print)
# ---------------------------------------------------------------------------
CLR = 0.3  # servo case in its pocket, part to part
TAB_CLR = 0.2  # each face of a tab in its groove
HORN_CLR = 0.15  # horn in its pocket: a light press, nylon gives
SLIT = 0.8  # slots that free a flexure inside one part (> 0.6 so it does not fuse)
RUN = 0.4  # running gap between parts that move against each other

# ---------------------------------------------------------------------------
# Horn envelope (common/fixtures.horn): hub 7 x 4 from 0.5 above the case,
# arm 1.6 thick at the hub's top.
# ---------------------------------------------------------------------------
HUB_Z0, HUB_H, HORN_ARM_T = 0.5, 4.0, 1.6
HORN_TOP = HUB_Z0 + HUB_H  # 4.5 above the case top
HORN_ARM_BOT = HORN_TOP - HORN_ARM_T  # 2.9

# ---------------------------------------------------------------------------
# Base and pan
# ---------------------------------------------------------------------------
FLOOR_T = 3.0
PAN_BASE_Z = FLOOR_T + 0.5  # pan servo hangs on its tabs, 0.5 above the floor
Z_PAN_TOP = PAN_BASE_Z + SERVO.height  # case top, servo frame z 0
Z_TAB_UNDER = PAN_BASE_Z + SERVO.tab_underside
POCKET_WALL = 2.1
YOKE_B = Z_PAN_TOP + HORN_ARM_BOT  # yoke underside = horn arm underside
YOKE_T = YOKE_B + HORN_ARM_T + 1.6  # 1.6 floor over the horn pocket
RING_TOP = YOKE_B - 0.3  # thrust face

_cx = SHAFT_FROM_END - SERVO.length / 2
_tab_corner = math.hypot(_cx - SERVO.tab_span / 2, SERVO.width / 2)
# the lead's boot sticks 6 mm out of the case's -X end (fixtures estimate)
_boot_corner = math.hypot(SHAFT_FROM_END - SERVO.length - 6.0, 2.0)
# the servo drops in through the ring, tabs and boot first
RING_RI = round(max(_tab_corner + 0.5, _boot_corner + 0.35), 2)
RING_RG = RING_RI + 1.8  # ring outer below the lip (the groove the hooks sit in)
RING_RL = RING_RG + 1.2  # lip outer
LIP_UNDER = RING_TOP - 12.0  # lip underside: the fingers catch here

FINGER_RI = RING_RL + CLR
FINGER_T = 1.8
FINGER_RO = FINGER_RI + FINGER_T
NOSE_RI = RING_RG + CLR
NOSE_H = 2.6
NOSE_TOP = LIP_UNDER - 0.2
FINGER_W = 11.0  # chord width
FINGER_ANGLES = [45, 135, 225, 315]
PLATE_R = math.hypot(FINGER_RO, FINGER_W / 2) + 0.6

M3_HOLES = [(-6.0, 31.0), (-6.0, -31.0)]  # world xy; drill the chassis 3.2 mm
EAR_T = FLOOR_T

# ---------------------------------------------------------------------------
# Tilt
# ---------------------------------------------------------------------------
CHEEK_IN = 14.0  # cradle cheeks' inner faces, |y|; the board edge is at 14.75
CHEEK_T = 3.6
CHEEK_OUT = CHEEK_IN + CHEEK_T
GROOVE_FLOOR = fx.HEAD_SIZE / 2 + 0.35  # board edge groove bottom, |y|
TILT_TOP_Y = -CHEEK_OUT - HORN_ARM_BOT  # tilt servo case top (servo z 0), world y
ARM_IN = CHEEK_OUT + RUN  # yoke +Y arm inner face
ARM_T = 3.0  # springs out ~4.5 mm to let the cradle's pivot stub in (PA11)
STUB_IN = 2.5  # pivot stub's engagement in the arm
ARM_OUT = ARM_IN + ARM_T
PIVOT_D = 6.0
PIVOT_HOLE = PIVOT_D + 0.4
PEG_D, PEG_TIP_D = 1.9, 1.3  # pegs into the head's 2.2 mm M2 holes

# The tilt axis in the head's board frame: midway between the XIAO stack's front
# and the J2 keep-out's back, so the head's swept radius is smallest; at the
# board's mid-height.
AX = (fx.HEAD_THICK + fx.XIAO_STACK[0] - fx.J2_KEEPOUT[0]) / 2  # 5.8
AZ = fx.HEAD_SIZE / 2
# The J2 keep-out's lowest point over tilt +-45 sits this far below the axis
# (computed in _j2_drop()); the yoke's top stays under it.
Z_TILT = None  # set below

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos(min(x0, x1), min(y0, y1), min(z0, z1)) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), align=(MIN, MIN, MIN)
    )


def cyl_z(r, z0, z1, x=0.0, y=0.0) -> Part:
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(CEN, CEN, MIN))


def cyl_y(r, y0, y1, x=0.0, z=0.0) -> Part:
    """Cylinder along +Y from y0 to y1."""
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=(CEN, CEN, MIN))


def cyl_x(r, x0, x1, y=0.0, z=0.0) -> Part:
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=(CEN, CEN, MIN))


def prism_yz(pts, x0, x1) -> Part:
    """Polygon (y, z) extruded along +X from x0 to x1."""
    p = extrude(Plane.YZ * Polygon(*pts, align=None), amount=x1 - x0)
    return Pos(x0 - p.bounding_box().min.X, 0, 0) * p


def prism_xz(pts, w) -> Part:
    """Polygon (x, z) extruded symmetrically along Y to a total width w."""
    return extrude(Plane.XZ * Polygon(*pts, align=None), amount=w / 2, both=True)


def _j2_drop() -> float:
    """Lowest z (below the tilt axis) the J2 keep-out reaches over tilt +-45."""
    xd, _yd, zd = fx.J2_KEEPOUT
    zc = fx.J2_CENTRE_YZ[1]
    corners = [(x, z) for x in (-xd, 0.0) for z in (zc - zd / 2, zc + zd / 2)]
    low = 0.0
    for t in range(-45, 46):
        a = math.radians(t)
        for x, z in corners:
            rx, rz = x - AX, z - AZ
            low = min(low, rx * math.sin(a) + rz * math.cos(a))
    return -low


Z_TILT = round(YOKE_T + _j2_drop() + 0.6, 2)
HEAD_LOC = Pos(-AX, 0, Z_TILT - AZ)  # board frame -> world at tilt 0


def tilt_servo_loc() -> Location:
    """Servo frame -> world: shaft along +Y on the tilt axis, long axis up
    (servo x -> world +Z, servo y -> world +X, servo z -> world +Y)."""
    return Location(
        Plane(origin=(0, TILT_TOP_Y, Z_TILT), x_dir=(0, 0, 1), z_dir=(0, 1, 0))
    )


# ---------------------------------------------------------------------------
# servo snap cradle, in the servo frame: two tab grooves that open toward +y_s
# (the slide-in direction), a back rail, and a flexure finger with a nose in
# the lower groove that clicks in front of the tab.
# ---------------------------------------------------------------------------


def slide_cradle(spec: fx.ServoSpec, side_w: float = 2.4) -> Part:
    cx = spec.shaft_from_end - spec.length / 2
    tab_lo = cx - spec.tab_span / 2  # tab ends, servo x
    tab_hi = cx + spec.tab_span / 2
    body_lo = spec.shaft_from_end - spec.length
    body_hi = spec.shaft_from_end
    zu = -(spec.height - spec.tab_underside)  # tab underside, servo z
    zo = zu + spec.tab_thickness
    w2 = spec.width / 2
    g0, g1 = zu - TAB_CLR, zo + TAB_CLR  # groove
    led0, lip1 = g0 - 2.0, g1 + 2.0
    end_t = 2.4  # 1.6 left beside the finger's slit
    back = -w2 - CLR - 2.3
    front = w2 + side_w + 0.5
    blocks = []
    for lo, hi, outer in (
        (body_hi + CLR, tab_hi + TAB_CLR + end_t, tab_hi + TAB_CLR),
        (tab_lo - TAB_CLR - end_t, body_lo - CLR, tab_lo - TAB_CLR),
    ):
        blk = box(lo, hi, back, front, led0, lip1)
        # the groove, open at +y_s, closed by the block's back wall
        blk -= box(lo - 1, hi + 1, -w2 - CLR, front + 1, g0, g1)
        # keep the end wall: re-add it (the groove cut ran through it)
        if outer > 0:
            blk += box(outer, hi, back, front, led0, lip1)
        else:
            blk += box(lo, outer, back, front, led0, lip1)
        blocks.append(blk)
    rail = box(
        tab_lo - TAB_CLR - end_t, tab_hi + TAB_CLR + end_t, back, -w2 - CLR, led0, lip1
    )
    cradle = blocks[0] + blocks[1] + rail
    # flexure finger: free the lower block's ledge from its end wall
    f_lo, f_hi = tab_lo - TAB_CLR, body_lo - CLR
    cradle -= box(f_lo - SLIT, f_lo, -w2 + 2.0, front + 1, led0 - 0.1, g0)
    # ... and from the rail side so it is a beam rooted at the back
    nose_y0 = w2 + TAB_CLR
    # nose: a vertical catch face behind the tab, a flat top, a lead-in ramp in front
    h = 0.8
    nose = prism_yz(
        [
            (nose_y0, g0 - 0.01),
            (nose_y0, g0 + h),
            (nose_y0 + side_w - 0.9, g0 + h),
            (nose_y0 + side_w, g0 - 0.01),
        ],
        f_lo + 0.3,
        f_hi - 0.3,
    )
    cradle += nose
    return cradle


def horn_pocket(kind: str, clr: float = HORN_CLR, through: float = 0.0) -> Part:
    """Pocket for a stock horn in the servo frame, from the arm's underside up to
    the horn top, plus an optional screw hole on the axis to ``through`` mm."""
    z0, z1 = HORN_ARM_BOT - 0.01, HORN_TOP
    p = cyl_z(3.5 + clr, z0, z1)
    if kind == "single":
        p += box(-clr, 20 + clr, -2.75 - clr, 2.75 + clr, z0, z1)
    elif kind == "cross":
        p += box(-16 - clr, 16 + clr, -2.5 - clr, 2.5 + clr, z0, z1)
        p += box(-2.5 - clr, 2.5 + clr, -16 - clr, 16 + clr, z0, z1)
    else:
        raise ValueError(kind)
    if through:
        p += cyl_z(2.25, z0, HORN_TOP + through)
    return p


# ---------------------------------------------------------------------------
# printed parts, world pose at pan 0 tilt 0
# ---------------------------------------------------------------------------


def make_base() -> Part:
    # floor with two M3 ears
    floor = cyl_z(RING_RG + 1.0, 0, FLOOR_T)  # a foot proud of the ring
    for x, y in M3_HOLES:
        s = 1 if y > 0 else -1
        floor += box(x - 5, x + 5, 0, y + s * 5.5, 0, EAR_T)
        floor += cyl_z(5.0, 0, EAR_T, x, y + s * 0.5)
    for x, y in M3_HOLES:
        floor -= cyl_z(1.7, -1, EAR_T + 1, x, y)
    ring = cyl_z(RING_RG, FLOOR_T - 0.01, RING_TOP)
    ring += cyl_z(RING_RL, LIP_UNDER, RING_TOP - 1.0)
    ring += Pos(0, 0, RING_TOP - 1.0) * Cone(
        RING_RL, RING_RL - 0.8, 1.0, align=(CEN, CEN, MIN)
    )
    ring -= cyl_z(RING_RI, FLOOR_T - 1, RING_TOP + 1)
    # pan servo pocket walls: the tabs rest on their tops
    x0 = SHAFT_FROM_END - SERVO.length - CLR
    x1 = SHAFT_FROM_END + CLR
    w = SERVO.width / 2 + CLR
    walls = box(
        x0 - POCKET_WALL,
        x1 + POCKET_WALL,
        -w - POCKET_WALL,
        w + POCKET_WALL,
        FLOOR_T - 0.01,
        Z_TAB_UNDER,
    )
    walls -= box(x0, x1, -w, w, FLOOR_T - 1, Z_TAB_UNDER + 1)
    # flat-faced boss round the lead exit, so the slot is cut through flat walls
    boss = box(-RING_RG - 1.0, -RING_RI + 0.5, -5.0, 5.0, 0, PAN_BASE_Z + 12.0)
    boss -= cyl_z(RING_RI, FLOOR_T - 1, RING_TOP)
    base = floor + ring + walls + boss
    # lead exit: the boot leaves the -X end low; slot through the wall and ring
    boot_z1 = PAN_BASE_Z + 9.5
    base -= box(-RING_RL - 2, x0 + 0.5, -2.6, 2.6, FLOOR_T, boot_z1)
    # ... and the rear pocket wall slotted to the top, so the boot drops past it
    base -= box(x0 - POCKET_WALL - 1, x0 + 0.5, -2.6, 2.6, FLOOR_T, Z_TAB_UNDER + 1)
    # lighten the floor inside the pocket (finger access to push the servo up)
    base -= cyl_z(4.0, -1, FLOOR_T + 1, -5.0, 0)
    return base


def _finger(angle: float) -> Part:
    """A flat cantilever (no curved faces, so no knife edges where it is cut)
    with an inward catch nose and a 45 degree lead-in under it."""
    z_root = YOKE_B + 0.01
    nb = NOSE_TOP - NOSE_H
    prof = [
        (FINGER_RO, nb),
        (FINGER_RO, z_root),
        (FINGER_RI, z_root),
        (FINGER_RI, NOSE_TOP),
        (NOSE_RI, NOSE_TOP),
        (NOSE_RI, NOSE_TOP - 1.5),
        (FINGER_RI, nb),
    ]
    return Rot(0, 0, angle) * prism_xz(prof, FINGER_W)


RIB_X0, RIB_X1 = -FINGER_RO - 7.5, -FINGER_RO + 4  # rear tongue with the ribbon hook
RIBBON_SLOT_X = (-FINGER_RO - 4.6, -FINGER_RO - 2.4)  # 2.2 wide, outside the fingers
RIBBON_SLOT_Y = (fx.J2_CENTRE_YZ[0] - 8.5, fx.J2_CENTRE_YZ[0] + 8.5)


def make_yoke() -> Part:
    tl = tilt_servo_loc()
    cradle_s = tl * slide_cradle(SERVO)
    cb = cradle_s.bounding_box()
    plate = cyl_z(PLATE_R, YOKE_B, YOKE_T)
    plate += box(cb.min.X - 1, cb.max.X + 1, cb.min.Y - 1, 0, YOKE_B, YOKE_T)
    # rear tongue carrying the ribbon hook
    y0, y1 = RIBBON_SLOT_Y
    plate += box(RIB_X0, RIB_X1, y0 - 2.5, y1 + 2.5, YOKE_B, YOKE_T)
    plate -= box(*RIBBON_SLOT_X, y0, y1 + 3.0, YOKE_B - 1, YOKE_T + 1)
    # servo lead clip at the -Y edge: a slot the flat lead presses into sideways
    ly = cb.min.Y - 1
    clip = box(-4.5, 4.5, ly - 4.6, ly + 0.01, YOKE_B, YOKE_T)
    clip -= box(-1.8, 1.8, ly - 3.0, ly - 1.6, YOKE_B - 1, YOKE_T + 1)
    clip -= box(-0.45, 0.45, ly - 4.7, ly - 2.9, YOKE_B - 1, YOKE_T + 1)
    plate += clip
    # pan horn pocket from below, screw access from above
    plate -= Pos(0, 0, Z_PAN_TOP) * horn_pocket("cross", through=3.0)
    yoke = plate
    for a in FINGER_ANGLES:
        yoke += _finger(a)
    # tilt servo cradle and its pillar down to the plate
    yoke += cradle_s
    yoke += box(cb.min.X, cb.max.X, cb.min.Y, cb.max.Y, YOKE_T - 0.01, cb.min.Z + 0.01)
    # +Y pivot arm
    arm = box(-6, 6, ARM_IN, ARM_OUT, YOKE_T - 0.01, Z_TILT)
    arm += cyl_y(6.6, ARM_IN, ARM_OUT, 0, Z_TILT)
    arm -= cyl_y(PIVOT_HOLE / 2, ARM_IN - 1, ARM_OUT + 1, 0, Z_TILT)
    # lead-in on the hole's inner edge
    arm -= (
        Pos(0, ARM_IN - 0.01, Z_TILT)
        * Rot(-90, 0, 0)
        * Cone(PIVOT_HOLE / 2 + 0.6, PIVOT_HOLE / 2, 0.6, align=(CEN, CEN, MIN))
    )
    yoke += arm
    return yoke


def _cradle_board() -> Part:
    """Head cradle in the head's board frame (board x 0..1.6, y +-14.75, z 0..29.5)."""
    t = fx.HEAD_THICK
    x0, x1 = -3.0, AX + 5.5
    zb = -1.5
    c = None
    for s in (1, -1):
        cheek = box(x0, x1, s * CHEEK_IN, s * CHEEK_OUT, zb, fx.HEAD_SIZE)
        # board-edge groove, open at the top
        cheek -= box(
            -0.15, t + 0.15, s * (CHEEK_IN - 1), s * GROOVE_FLOOR, 0, fx.HEAD_SIZE + 1
        )
        if s < 0:
            # horn arm points down: extend the -Y cheek to cover it
            cheek += box(
                AX - 4.8, AX + 4.8, s * CHEEK_IN, s * CHEEK_OUT, AZ - 22.2, zb + 0.01
            )
            # clear the J2 plug behind the board's lower -Y corner
            cheek -= box(x0 - 1, -0.15, -CHEEK_IN + 0.01, -GROOVE_FLOOR, zb - 1, 8.5)
        c = cheek if c is None else c + cheek
    c += box(-3.0, -0.2, -CHEEK_IN - 0.01, CHEEK_IN + 0.01, 8.0, 11.0)  # back bar
    c += box(t + 0.15, t + 2.6, -CHEEK_IN - 0.01, CHEEK_IN + 0.01, zb, 3.5)  # front bar
    # latch tongues with pegs into the M2 holes
    for y, z in fx.HEAD_HOLES_YZ:
        c += box(-1.8, -0.2, y - 2.0, y + 2.0, 10.99, z + 2.0)
        # peg with a conical tip: the board's bottom edge pushes it back
        c += cyl_x(PEG_D / 2, -0.21, 0.6, y, z)
        c += (
            Pos(0.6 - 0.01, y, z)
            * Rot(0, 90, 0)
            * Cone(PEG_D / 2, PEG_TIP_D / 2, t - 0.7, align=(CEN, CEN, MIN))
        )
    return c


def make_cradle() -> Part:
    c = HEAD_LOC * _cradle_board()
    tl = tilt_servo_loc()
    # single-arm horn pointing down, pressed into the -Y cheek's outer face
    c -= tl * Rot(0, 0, 180) * horn_pocket("single", through=CHEEK_T)
    # pivot stub on the +Y cheek, running in the yoke arm's hole
    stub_end = ARM_IN + STUB_IN
    c += cyl_y(PIVOT_D / 2, CHEEK_OUT - 0.01, stub_end - 0.5, 0, Z_TILT)
    c += (
        Pos(0, stub_end - 0.51, Z_TILT)
        * Rot(-90, 0, 0)
        * Cone(PIVOT_D / 2, PIVOT_D / 2 - 0.5, 0.5, align=(CEN, CEN, MIN))
    )
    if EYEBROWS:
        c += brow_mount_on_cradle()
    return c


# ---------------------------------------------------------------------------
# Eyebrows: two SG90s standing side by side on a deck behind the board's top
# edge, shafts forward (servo z -> world +X), shaft end down (servo x -> -Z).
# Each slides in sideways (outboard) along its tab grooves and clicks; the two
# share one back rail on the centre line.
# ---------------------------------------------------------------------------
BROW_FACE_X = 4.0  # SG90 case top (shaft face), board frame x
DECK_T = 2.0
DECK_Z0 = fx.HEAD_SIZE - 0.1  # deck underside, board frame (on the cheeks' tops)
BROW_RAIL = 2.3  # = slide_cradle's back wall
BROW_SHAFT_Y = BROW.width / 2 + CLR + BROW_RAIL / 2  # rails coincide on y = 0
_bcx = BROW.shaft_from_end - BROW.length / 2
BROW_SHAFT_Z = DECK_Z0 + DECK_T + (_bcx + BROW.tab_span / 2) + TAB_CLR + 2.0
BROW_ANGLE = 10.0  # resting brow tilt, deg: a little stern (firmware moves it)
BROW_LEN = 17.0


def brow_servo_loc(s: int) -> Location:
    """SG90 frame -> world (s = +1 left brow, -1 right)."""
    return HEAD_LOC * Location(
        Plane(
            origin=(BROW_FACE_X, s * BROW_SHAFT_Y, BROW_SHAFT_Z),
            x_dir=(0, 0, -1),
            z_dir=(1, 0, 0),
        )
    )


def brow_horn_rot(s: int) -> Location:
    """Horn arm outboard (+-world Y), outer end raised BROW_ANGLE (stern)."""
    return Rot(0, 0, s * (90 + BROW_ANGLE))


def brow_mount_on_cradle() -> Part:
    """The brow pod, printed as part of the cradle: a deck on the cheeks' tops
    behind the board, two slide cradles on it, and two gusset ribs."""
    pod = None
    for s in (1, -1):
        sc = slide_cradle(BROW)
        if s < 0:
            sc = mirror(sc, about=Plane.XZ)
        sc = brow_servo_loc(s) * sc
        pod = sc if pod is None else pod + sc
    pb = pod.bounding_box()
    hx, hz = HEAD_LOC.position.X, HEAD_LOC.position.Z
    # deck: stops 0.3 behind the board's back face so the board still drops in
    deck = box(
        pb.min.X - 0.5,
        hx - 0.3,
        -CHEEK_OUT,
        CHEEK_OUT,
        hz + DECK_Z0,
        hz + DECK_Z0 + DECK_T + 0.01,
    )
    # gussets down the cheeks' back edges (clear of the latch tongues)
    for s in (1, -1):
        deck += box(
            pb.min.X - 0.5,
            hx - 2.9,
            s * CHEEK_IN,
            s * CHEEK_OUT,
            hz + 20.0,
            hz + DECK_Z0 + 0.01,
        )
    return pod + deck


def make_brow(s: int) -> Part:
    """A bushy brow bar pressed onto an SG90 single-arm horn."""
    z0, z1 = HORN_ARM_BOT, HORN_TOP + 2.8
    hw = 2.75 + HORN_CLR + 1.6
    br = cyl_z(3.5 + HORN_CLR + 1.6, z0, z1) + box(0, BROW_LEN + 3.0, -hw, hw, z0, z1)
    # bushy: a raised ridge along the upper edge (servo +y at the left brow)
    br += box(1.0, BROW_LEN + 3.0, s * (hw - 2.0), s * hw, z1 - 0.01, z1 + 1.6)
    br -= horn_pocket("single", through=1.2)
    return brow_servo_loc(s) * brow_horn_rot(s) * br


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------


def printed_parts() -> dict[str, Part]:
    parts = {
        "base": make_base(),
        "yoke": make_yoke(),
        "cradle": make_cradle(),
    }
    if EYEBROWS:
        parts["brow-left"] = make_brow(1)
        parts["brow-right"] = make_brow(-1)
    return parts


COLOURS = {
    "base": "#3d3f44",
    "yoke": "#3a7ca5",
    "cradle": "#d9822b",
    "brow-left": "#1e1e1e",
    "brow-right": "#1e1e1e",
}

_CACHE: dict[str, Part] = {}


def _parts() -> dict[str, Part]:
    if not _CACHE:
        _CACHE.update(printed_parts())
    return _CACHE


def _m3(x, y) -> Part:
    return cyl_z(2.85, EAR_T, EAR_T + 1.65, x, y) + cyl_z(1.5, -3.0, EAR_T + 0.01, x, y)


def assembly(pan_deg: float, tilt_deg: float) -> list[Body]:
    P = _parts()
    pan = Rot(0, 0, pan_deg)
    # positive tilt = nose up
    tilt = Pos(0, 0, Z_TILT) * Rot(0, -tilt_deg, 0) * Pos(0, 0, -Z_TILT)
    pt = pan * tilt
    tl = tilt_servo_loc()
    bodies = [
        Body("base", P["base"], "base", printed=True, colour=COLOURS["base"]),
        Body(
            "pan-servo",
            Pos(0, 0, Z_PAN_TOP) * fx.servo(SERVO),
            "base",
            allow={"pan-horn"},
        ),
    ]
    for i, (x, y) in enumerate(M3_HOLES):
        bodies.append(Body(f"m3-{i}", _m3(x, y), "base"))
    bodies += [
        Body("yoke", pan * P["yoke"], "pan", printed=True, colour=COLOURS["yoke"]),
        Body("pan-horn", pan * Pos(0, 0, Z_PAN_TOP) * fx.horn("cross", SERVO), "pan"),
        Body("tilt-servo", pan * tl * fx.servo(SERVO), "pan", allow={"tilt-horn"}),
        Body("tilt-horn", pt * tl * Rot(0, 0, 180) * fx.horn("single", SERVO), "tilt"),
        Body(
            "cradle", pt * P["cradle"], "tilt", printed=True, colour=COLOURS["cradle"]
        ),
        Body("head", pt * HEAD_LOC * fx.head(screws=False), "tilt"),
    ]
    if EYEBROWS:
        for s, name in ((1, "left"), (-1, "right")):
            loc = brow_servo_loc(s)
            bodies.append(Body(f"brow-servo-{name}", pt * loc * fx.servo(BROW), "tilt"))
            bodies.append(
                Body(
                    f"brow-horn-{name}",
                    pt * loc * brow_horn_rot(s) * fx.horn("single", BROW),
                    "tilt",
                )
            )
            bodies.append(
                Body(
                    f"brow-{name}",
                    pt * P[f"brow-{name}"],
                    "tilt",
                    printed=True,
                    colour=COLOURS[f"brow-{name}"],
                )
            )
    return bodies


def main() -> None:
    out = HERE / "output"
    out.mkdir(exist_ok=True)
    print(f"tilt axis z {Z_TILT}, pan servo top z {Z_PAN_TOP}, yoke top z {YOKE_T}")
    for name, part in printed_parts().items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(
            part, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1
        )
        bb = part.bounding_box()
        print(
            f"{name:12s} {part.volume / 1000:6.2f} cm3  valid={part.is_valid}  "
            f"bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}"
        )


if __name__ == "__main__":
    main()
