"""The Workhorse: a conventional two-axis U-bracket pan-tilt for two MG90D servos.

Four structural parts, all MJF PA12-HP (no supports, takes self-tapping screws,
nylon-on-brass pivot), plus an optional brow bracket and two brows for Teuvo.

    base          pan servo pocket + chassis flange (4 x M3)
    yoke          U-bracket on the pan horn: plate + two cheeks; the +Y cheek
                  holds the tilt servo, the -Y cheek is the plain bearing
    cradle        holds head 1 by its two M2 holes; +Y arm carries the tilt
                  horn, -Y arm the M3 pivot screw + brass sleeve + nut
    brow_bracket  optional: a flat plate behind the cradle holding two SG90s
    brow          optional, print 2: Teuvo's eyebrows, snap over the SG90 horns

Parts are exported in their WORLD pose at pan 0 / tilt 0 (MJF needs no
supports, so the print orientation does not matter; JLC3DP orients them).

World frame (common/fixtures.py): +X forward, +Y left, +Z up, mm; z 0 is the
top of the acrylic chassis plate. Pan axis = world Z through x 0, y 0. Tilt
axis = world Y through x 0, z ZT.

Run ``python design.py`` to write output/<part>.step and .stl.
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

from build123d import (
    Align,
    Box,
    Cylinder,
    Part,
    Pos,
    Polygon,
    RegularPolygon,
    Rot,
    export_step,
    export_stl,
    extrude,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))  # hardware/pan-tilt, for common.*

from common import fixtures as fx  # noqa: E402
from common.check import Body  # noqa: E402

# --- Caliper these before ordering (README "Measure first") ------------------
# The MG90D's published A-F are used as-is; the rest are fixture estimates.
MG90D = dataclasses.replace(
    fx.MG90D,
    tab_span=31.5,  # E: across the tabs. Sets the tab screw pilots.
    tab_underside=19.8,  # F: base to tab underside. Sets the base pocket depth
    tab_thickness=2.5,  # estimate
    tab_hole_inset=2.3,  # estimate: tab hole centre from the tab end
    shaft_from_end=6.0,  # estimate: shaft axis from the case's near end
)
SG90 = fx.SG90
EYEBROWS = True  # include the brow bracket, two SG90s and brows in assembly()

# --- Fits (MJF PA12-HP: JLC3DP states +/-0.3 mm up to 100 mm) ---------------
FIT = 0.3  # per side, servo body in its pocket/window
PILOT_SERVO = 1.5  # servo's own 2 mm self-tapping screws (tabs, horn arms)
PILOT_M2 = 1.7  # M2 machine screw self-tapping into PA12 (head board)
CLEAR_M2 = 2.3
CLEAR_M3 = 3.4
BORE = 4.1  # plain bearing for the 4.0 mm brass sleeve; ream with a 4.1 drill
NUT_AF = 5.8  # M3 nut 5.5 AF + fit
HORN_FIT = 0.3  # horn pocket clearance per side

# --- Layout -----------------------------------------------------------------
FLOOR = 3.0  # base floor and flange thickness
PAN_GAP = 1.0  # below the pan servo's case
PAN_TAB_Z = FLOOR + PAN_GAP + MG90D.tab_underside  # tab underside, world z
PAN_TOP = FLOOR + PAN_GAP + MG90D.height  # pan servo case top, world z
HORN_ARM_Z0 = 0.5 + 4.0 - 1.6  # fixtures.horn(): arm bottom above the case top
PLATE_Z0 = PAN_TOP + HORN_ARM_Z0 - 0.4  # yoke plate underside
PLATE_Z1 = PAN_TOP + 4.5 + 4.0  # yoke plate top: 4 mm solid above the horn
HEAD_DROP = 24.5  # tilt axis to plate top: clears the J2 ribbon plug at +/-45 deg
ZT = PLATE_Z1 + HEAD_DROP  # tilt axis height

AXIS_IN_HEAD = (
    4.0,
    12.0,
)  # tilt axis in head-1 coords (x from board back, z from bottom)

# Cradle (head frame: x 0..1.6 is the board, y centred, z 0 its bottom edge)
ARM_IN = fx.HEAD_SIZE / 2 + 0.75  # arm inner faces, 0.75 mm off the board edges
ARM_P = 5.0  # +Y arm thickness (horn side)
ARM_N = 7.0  # -Y arm thickness (nut side)
BOSS = 0.5  # raised boss on the -Y arm face, the sleeve's seat
BACK_T = 5.0  # back plate behind the board
BACK_Z0 = fx.J2_CENTRE_YZ[1] + fx.J2_KEEPOUT[2] / 2 + 1.0  # above the J2 plug
GAP = 1.0  # arm to cheek

# Yoke
Y_ARM_P = ARM_IN + ARM_P  # +Y arm outer face, world y
Y_CASE = (
    Y_ARM_P + HORN_ARM_Z0
)  # tilt servo case top, world y (horn arm flush in pocket)
CHEEK_P = (
    Y_ARM_P + GAP,
    Y_CASE + MG90D.height - MG90D.tab_underside - MG90D.tab_thickness,
)
Y_ARM_N = -(ARM_IN + ARM_N)
CHEEK_N = (Y_ARM_N - BOSS - 0.5 - 6.0, Y_ARM_N - BOSS - 0.5)
CHEEK_TOP_P = ZT + MG90D.width / 2 + FIT + 3.2
CHEEK_TOP_N = ZT + 8.0
PLATE_X = (-32.0, 14.0)

# Ribbon (14-way, 1.0 mm pitch for J2's 2.0 mm IDC: 14 mm wide, ~1 mm thick).
# It drops through a slot in the yoke plate close behind the pan axis, flat
# face toward the axis, folds 45 deg just above the flange into an edgewise
# run, wraps round the base's round rear (the drum) toward +Y and leaves
# forward along the base's +Y side. See README "Ribbon path".
RIBBON_W = 14.0
RIBBON_R = 16.4  # drop plane's distance behind the pan axis
RIBBON_SLOT = (-(RIBBON_R + 1.2), -(RIBBON_R - 1.2), -8.0, 8.0)  # x0 x1 y0 y1
RIBBON_TIE = (-21.2, -19.8, -1.5, 1.5)  # cable-tie slot behind a 2.2 mm bar

# Base. The pan servo is turned shaft-rearward (body and lead forward) so the
# base's rear end is a drum of radius DRUM_R round the pan axis: the ribbon's
# drop can then sit 16 mm from the axis instead of 27.5 and never crosses the
# base in the +/-90 sweep.
BASE_X = (-12.4, 23.0)
BASE_Y = 9.0
DRUM_R = 12.4
FLANGE_X = (-23.5, 23.0)
FLANGE_Y = 22.5
GUIDE_R = (20.0, 22.0)  # ribbon guide fence round the rear 120 deg
GUIDE_HALF_DEG = 56.0  # arc, plus a 2.5 mm straight tangent tail at each end
GUIDE_TAIL = 2.5
GUIDE_TOP = 19.0
MOUNT_HOLES = [(x, y) for x in (-19.0, 16.0) for y in (-18.5, 18.5)]  # M3, 35 x 37 mm

# Pivot hardware (stand-ins): M3 x 14 socket head, 4/3.1 brass tube sleeve 7.0 long
SLEEVE_L = 7.0
SCREW_L = 14.0

# Tilt horn arm screws: PB2 x 4 pan head (DIN 7981 ST2.2: head 4.0 x 1.6),
# driven from INSIDE the cradle through a 2.2 clearance hole and tapping into
# the horn's own arm holes (drilled to 1.6). The head sits in a 1 mm
# counterbore on the +Y arm's inner face, the tip ends flush with the horn's
# servo-side face, so nothing stands in the 1 mm arm-to-cheek gap.
HORN_SCREW_R = (11.0, 17.5)  # hole radii on the single horn (measure yours)
PB2_HEAD = (4.0, 1.6)  # diameter, height
PB2_CBORE = 1.0
PAN_HORN_SCREW_R = 14.0  # double horn, driven up into the yoke

# XIAO socket pin tails on head 1's back face: relief pocket in the back plate
SOCKET_ROW_Y = (fx.XIAO_CENTRE_YZ[0] - 7.62, fx.XIAO_CENTRE_YZ[0] + 7.62)
SOCKET_ROW_Z = (fx.XIAO_CENTRE_YZ[1] - 7.62, fx.XIAO_CENTRE_YZ[1] + 7.62)
TAIL_R = 1.3  # pin tail + solder fillet radius, plus tolerance
RELIEF_D = 2.5  # pocket depth into the 5 mm back plate

# Brow bracket screws (M2 x 6) into blind pilots in the back plate
BROW_SCREW_YZ = [(-12.0, 18.0), (12.0, 18.0)]
BROW_BRACKET_Z0 = 14.5  # clear of the tilt horn screw heads below

# Eyebrows: SG90 shaft forward, long axis vertical, shaft near the bottom
BROW_CASE_X = 8.1  # SG90 case top, head x
BROW_SHAFT_Z = 41.0  # head z
BROW_Y = 7.5
BROW_H = 10.4  # brow height: 2 mm walls round the horn pocket
BROW_PLATE_X = (
    BROW_CASE_X - (SG90.height - SG90.tab_underside) - 4.0,
    BROW_CASE_X - (SG90.height - SG90.tab_underside),
)

C_STRUCT = "#5f6670"  # MJF grey
C_BROW = "#1d1d1f"


# --- helpers ----------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0
    )


def cyl_z(x, y, r, z0, z1) -> Part:
    return Pos(x, y, z0) * Cylinder(
        r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def cyl_y(x, z, r, y0, y1) -> Part:  # axis along Y from y0 to y1
    return (
        Pos(x, y1, z)
        * Rot(X=90)
        * Cylinder(r, y1 - y0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def cyl_x(y, z, r, x0, x1) -> Part:
    return (
        Pos(x0, y, z)
        * Rot(Y=90)
        * Cylinder(r, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def head_to_world(x, z):
    return x - AXIS_IN_HEAD[0], z - AXIS_IN_HEAD[1] + ZT


HEAD_LOC = Pos(-AXIS_IN_HEAD[0], 0, ZT - AXIS_IN_HEAD[1])

# servo frame -> world, at rest
PAN_LOC = Pos(0, 0, PAN_TOP) * Rot(Z=180)  # shaft rearward of the case centre
TILT_LOC = Pos(0, Y_CASE, ZT) * Rot(X=90)  # shaft points -Y, body outboard (+Y)
BROW_LOCS = [
    HEAD_LOC * Pos(BROW_CASE_X, s * BROW_Y, BROW_SHAFT_Z) * Rot(Y=90) for s in (1, -1)
]


def _servo_body_x(spec):
    """Case x range in the servo frame."""
    return spec.shaft_from_end - spec.length, spec.shaft_from_end


def wedge(phi0, phi1, z0, z1, r=80.0) -> Part:
    """A sector of the XY plane from angle phi0 to phi1 (deg, from +X, CCW)."""
    n = max(2, int(abs(phi1 - phi0) // 15) + 1)
    pts = [(0.0, 0.0)] + [
        (
            r * math.cos(math.radians(phi0 + (phi1 - phi0) * i / n)),
            r * math.sin(math.radians(phi0 + (phi1 - phi0) * i / n)),
        )
        for i in range(n + 1)
    ]
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=z1 - z0)


def ring(r0, r1, phi0, phi1, z0, z1) -> Part:
    return (cyl_z(0, 0, r1, z0, z1) - cyl_z(0, 0, r0, z0 - 1, z1 + 1)) & wedge(
        phi0, phi1, z0 - 1, z1 + 1
    )


# --- printed parts (world pose, pan 0 tilt 0) --------------------------------
def base() -> Part:
    s = MG90D
    flange = box(FLANGE_X[0], FLANGE_X[1], -FLANGE_Y, FLANGE_Y, 0, FLOOR)
    # body: flat sides from where they meet the drum, round rear (the drum)
    xr = -math.sqrt(DRUM_R**2 - BASE_Y**2)
    block = box(xr, BASE_X[1], -BASE_Y, BASE_Y, 0, PAN_TAB_Z)
    drum = cyl_z(0, 0, DRUM_R, 0, PAN_TAB_Z) & box(
        -DRUM_R - 1, 0, -DRUM_R - 1, DRUM_R + 1, -1, PAN_TAB_Z + 1
    )
    # flat on the -Y side where the open pocket meets it (no knife edge there;
    # the ribbon never wraps this part, see _ribbon_wrap)
    drum -= box(xr, 1, -DRUM_R - 1, -BASE_Y, -1, PAN_TAB_Z + 1)
    fence = ring(
        GUIDE_R[0], GUIDE_R[1], 180 - GUIDE_HALF_DEG, 180 + GUIDE_HALF_DEG, 0, GUIDE_TOP
    )
    # straight tails: a radial end on a faceted arc reads as a sliver to the
    # ray gate, and the tails keep the ribbon's edge off a sharp corner
    for sgn in (1, -1):
        y0, y1 = sorted((0.0, sgn * GUIDE_TAIL))
        tail = box(GUIDE_R[0], GUIDE_R[1], y0, y1, 0, GUIDE_TOP)
        fence += Rot(Z=180 + sgn * GUIDE_HALF_DEG) * tail
    # the +Y side carries on flat and tangent from the drum, so the ribbon's
    # edgewise run leaves it smoothly and lies along it to the front
    side = box(-0.5, BASE_X[1], BASE_Y - 0.01, DRUM_R, 0, PAN_TAB_Z)
    p = flange + block + drum + side + fence
    # servo turned 180 deg: case spans world x -shaft_from_end .. length - that
    bx0, bx1 = _servo_body_x(s)
    wx0, wx1 = -bx1, -bx0
    # The pocket is open on the -Y side: the servo slides in sideways (+Y) with
    # its tabs gliding over the rim, so the lead's root never has to pass a
    # wall. Both tab pilots sit in the full-width end walls.
    p -= box(
        wx0 - FIT, wx1 + FIT, -FLANGE_Y - 1, s.width / 2 + FIT, FLOOR, PAN_TAB_Z + 1
    )
    # lead exit: a slot through the +X end wall, open to the -Y side and down
    # through the floor, so the lead drops through a chassis hole or runs forward
    p -= box(
        wx1, BASE_X[1] + 1, -BASE_Y - 1, 2.75, FLOOR - 0.01, FLOOR + PAN_GAP + 10.0
    )
    p -= box(wx1, BASE_X[1] + 1, -2.75, 2.75, -1, FLOOR + 1)
    for x, _, _ in fx.servo_tab_holes(s):
        p -= cyl_z(-x, 0, PILOT_SERVO / 2, PAN_TAB_Z - 7.0, PAN_TAB_Z + 1)
    for x, y in MOUNT_HOLES:
        p -= cyl_z(x, y, CLEAR_M3 / 2, -1, FLOOR + 1)
    return p


def _horn_pocket(kind, rot_deg, depth_lo, depth_hi):
    """A horn's footprint as a cutter, in the servo frame, z from depth_lo to depth_hi."""
    f = HORN_FIT
    hub = cyl_z(0, 0, 3.5 + f, depth_lo, depth_hi)
    if kind == "double":
        arm = box(-20 - f, 20 + f, -2.75 - f, 2.75 + f, depth_lo, depth_hi)
    else:
        arm = box(-f, 20 + f, -2.75 - f, 2.75 + f, depth_lo, depth_hi)
    return Rot(Z=rot_deg) * (hub + arm)


def yoke() -> Part:
    plate = box(PLATE_X[0], PLATE_X[1], CHEEK_N[0], CHEEK_P[1], PLATE_Z0, PLATE_Z1)
    s = MG90D
    bx0, bx1 = _servo_body_x(s)
    tab_x0 = (bx0 + bx1) / 2 - s.tab_span / 2
    tab_x1 = (bx0 + bx1) / 2 + s.tab_span / 2
    cheek_p = box(
        tab_x0 - 3.0, tab_x1 + 3.0, CHEEK_P[0], CHEEK_P[1], PLATE_Z0, CHEEK_TOP_P
    )
    cheek_n = box(-9.0, 9.0, CHEEK_N[0], CHEEK_N[1], PLATE_Z0, CHEEK_TOP_N)
    # gussets: low ribs outside each cheek tie it to the plate's edge
    p = plate + cheek_p + cheek_n
    # pan horn (double, arms along world Y) captured in a pocket in the underside
    hz = PAN_TOP
    p -= Pos(0, 0, hz) * _horn_pocket("double", 90, PLATE_Z0 - hz - 1, 4.5 + 0.2)
    p -= cyl_z(0, 0, 2.8, PLATE_Z0 - 1, PLATE_Z1 + 1)  # horn screw / screwdriver access
    for y in (-PAN_HORN_SCREW_R, PAN_HORN_SCREW_R):  # driven up through the horn
        p -= cyl_z(0, y, PILOT_SERVO / 2, PLATE_Z0 - 1, PLATE_Z1 - 1.5)
    # tilt servo window in the +Y cheek (closed: the servo goes in from outside)
    p -= box(
        bx0 - FIT,
        bx1 + FIT,
        CHEEK_P[0] - 1,
        CHEEK_P[1] + 1,
        ZT - s.width / 2 - FIT,
        ZT + s.width / 2 + FIT,
    )
    for x, _, _ in fx.servo_tab_holes(s):  # tab pilots, 7 mm deep into the cheek
        p -= cyl_y(x, ZT, PILOT_SERVO / 2, CHEEK_P[1] - 7.0, CHEEK_P[1] + 1)
    # -Y cheek: plain bearing for the brass sleeve
    p -= cyl_y(0, ZT, BORE / 2, CHEEK_N[0] - 1, CHEEK_N[1] + 1)
    # ribbon slot close behind the pan axis; a cable tie through the small slot
    # behind it clamps the ribbon to the 2.2 mm bar between them
    for x0, x1, y0, y1 in (RIBBON_SLOT, RIBBON_TIE):
        p -= box(x0, x1, y0, y1, PLATE_Z0 - 1, PLATE_Z1 + 1)
    # tilt-servo lead tie slots at the +Y cheek's rear foot
    for y in (CHEEK_P[0] - 4.0, CHEEK_P[0] - 9.0):
        p -= box(-29.5, -27.0, y - 1.0, y + 1.0, PLATE_Z0 - 1, PLATE_Z1 + 1)
    return p


def cradle() -> Part:
    """Built in head coordinates, returned in world pose."""
    ax, az = AXIS_IN_HEAD
    top = fx.HEAD_SIZE
    back = box(-BACK_T, 0, Y_ARM_N, Y_ARM_P, BACK_Z0, top)
    arm_p = box(-BACK_T, ax + 10.0, ARM_IN, Y_ARM_P, az - 10.0, top)
    arm_p += box(
        ax - 24.0, -BACK_T + 0.01, ARM_IN, Y_ARM_P, az - 5.0, az + 5.0
    )  # horn tail
    arm_n = box(-BACK_T, ax + 10.0, Y_ARM_N, -ARM_IN, az - 10.0, top)
    boss = cyl_y(ax, az, 4.2, Y_ARM_N - BOSS, Y_ARM_N + 0.01)
    p = back + arm_p + arm_n + boss
    # head-1 M2 holes: self-tapping pilots straight through the back plate
    for y, z in fx.HEAD_HOLES_YZ:
        p -= cyl_x(y, z, PILOT_M2 / 2, -BACK_T - 1, 1)
    # relief under the XIAO sockets' pin tails and fillets: the board bears on
    # the plate only outside the two rows and along the top band by the screws
    p -= box(
        -RELIEF_D,
        1,
        SOCKET_ROW_Y[0] - TAIL_R,
        SOCKET_ROW_Y[1] + TAIL_R,
        BACK_Z0 - 1,
        SOCKET_ROW_Z[1] + TAIL_R,
    )
    # brow bracket screws (M2 x 6), blind from behind, stopping 1.2 mm short of
    # the board; the screw tip ends at x -2
    for y, z in BROW_SCREW_YZ:
        p -= cyl_x(y, z, PILOT_M2 / 2, -BACK_T - 1, -1.2)
    # tilt horn (single arm pointing rearward) in a pocket on the +Y arm face
    # horn frame: servo z axis = -Y; its flat top lies HORN_ARM_Z0+1.6 inboard of the case
    horn_cut = _horn_pocket("single", 180, 2.9 - 0.2, 4.5 + 0.2)
    p -= Pos(ax, Y_ARM_P + HORN_ARM_Z0, az) * Rot(X=90) * horn_cut
    p -= cyl_y(ax, az, 2.8, ARM_IN - 1, Y_ARM_P + 1)  # horn screw head + spline end
    for r in HORN_SCREW_R:  # horn arm screws, driven from inside the cradle
        p -= cyl_y(ax - r, az, CLEAR_M2 / 2 - 0.05, ARM_IN - 1, Y_ARM_P + 1)
        p -= cyl_y(ax - r, az, PB2_HEAD[0] / 2 + 0.2, ARM_IN - 1, ARM_IN + PB2_CBORE)
    # -Y pivot: M3 clearance through boss and arm, M3 nut in a hex pocket inside
    p -= cyl_y(ax, az, CLEAR_M3 / 2, Y_ARM_N - BOSS - 1, -ARM_IN + 1)
    nut = Rot(X=90) * extrude(RegularPolygon(NUT_AF / math.sqrt(3), 6), amount=3.0)
    p -= Pos(ax, -ARM_IN + 0.01, az) * Rot(Y=90) * nut  # flats vertical
    return HEAD_LOC * p


def brow_bracket() -> Part:
    """Flat plate behind the cradle: two SG90 windows above, two M2 holes below."""
    x0, x1 = BROW_PLATE_X
    s = SG90
    bx0, bx1 = _servo_body_x(s)  # servo x; servo +X maps to head -Z
    zt = BROW_SHAFT_Z - (bx0 - 6.0) + FIT + 2.5  # above the lead notch
    p = box(x0, x1, -15.3, 15.3, BROW_BRACKET_Z0, zt)
    # stay clear of the cradle's back plate (x -5..0) and the J2 keep-out
    for sy in (1, -1):
        y = sy * BROW_Y
        p -= box(
            x0 - 1,
            x1 + 1,
            y - s.width / 2 - FIT,
            y + s.width / 2 + FIT,
            BROW_SHAFT_Z - bx1 - FIT,
            BROW_SHAFT_Z - bx0 + FIT,
        )
        # the lead's root passes a notch above the window as the servo goes in
        # from the front; that leaves the upper tab bearing on the plate either
        # side of the notch, unscrewed (a dab of hot glue), the lower one screwed
        p -= box(
            x0 - 1,
            x1 + 1,
            y - 2.5,
            y + 2.5,
            BROW_SHAFT_Z - bx0,
            BROW_SHAFT_Z - (bx0 - 6.0) + FIT,
        )
        hx = max(h[0] for h in fx.servo_tab_holes(s))  # the lower tab
        p -= cyl_x(y, BROW_SHAFT_Z - hx, PILOT_SERVO / 2, x1 - 6.0, x1 + 1)
    for y, z in BROW_SCREW_YZ:
        p -= cyl_x(y, z, CLEAR_M2 / 2, x0 - 1, x1 + 1)
        # 1 mm counterbore: the M2 x 6 head seats at x0 + 1, its tip at x0 + 7
        p -= cyl_x(y, z, 2.1, x0 - 1, x0 + 1.0)
    # lighten the middle between the screws
    p -= box(x0 - 1, x1 + 1, -5.0, 5.0, 17.5, 26.0)
    return HEAD_LOC * p


def brow_local() -> Part:
    """One eyebrow in the SG90 servo frame, arm along +servo Y (rotate for the other)."""
    z0 = 2.9 - 0.2  # pocket floor sits at the horn arm's underside... from the back
    bar = box(-BROW_H / 2, BROW_H / 2, -6.0, 23.5, z0, z0 + 4.0)
    cut = Rot(Z=90) * (
        cyl_z(0, 0, 3.5 + HORN_FIT, z0 - 1, 4.5 + 0.2)
        + box(
            -HORN_FIT,
            20 + HORN_FIT,
            -2.75 - HORN_FIT,
            2.75 + HORN_FIT,
            z0 - 1,
            4.5 + 0.2,
        )
    )
    b = bar - cut
    b -= cyl_z(0, 0, 1.2, z0, z0 + 5)  # horn screw through the brow
    return b


def brow(side: int = 1) -> Part:
    loc = BROW_LOCS[0 if side > 0 else 1]
    return loc * (Rot(Z=0 if side > 0 else 180) * brow_local())


def printed_parts() -> dict[str, Part]:
    parts = {"base": base(), "yoke": yoke(), "cradle": cradle()}
    if EYEBROWS:
        parts["brow_bracket"] = brow_bracket()
        parts["brow"] = brow(1)
    return parts


# --- assembly -----------------------------------------------------------------
_CACHE: dict[str, Part] = {}


def _cached(name, fn):
    if name not in _CACHE:
        _CACHE[name] = fn()
    return _CACHE[name]


def _pivot_hw() -> tuple[Part, Part]:
    _ax, _az = AXIS_IN_HEAD
    y_head = Y_ARM_N - BOSS - SLEEVE_L
    sleeve = cyl_y(0, ZT, 2.0, y_head, Y_ARM_N - BOSS) - cyl_y(
        0, ZT, 1.55, y_head - 1, Y_ARM_N
    )
    screw = cyl_y(0, ZT, 2.75, y_head - 3.0, y_head) + cyl_y(
        0, ZT, 1.5, y_head, y_head + SCREW_L
    )
    return sleeve, screw


def _tilt_horn_screws() -> Part:
    """PB2 x 4 pan heads on the +Y arm's inner face (head frame -> world)."""
    ax, az = AXIS_IN_HEAD
    seat = ARM_IN + PB2_CBORE
    heads = [
        cyl_y(ax - r, az, PB2_HEAD[0] / 2, seat - PB2_HEAD[1], seat)
        for r in HORN_SCREW_R
    ]
    return HEAD_LOC * (heads[0] + heads[1])


def _pan_horn_screws() -> Part:
    """PB2 heads under the double horn's arms, driven up into the yoke."""
    z1 = PAN_TOP + HORN_ARM_Z0
    a, b = (
        cyl_z(0, y, PB2_HEAD[0] / 2, z1 - PB2_HEAD[1], z1)
        for y in (-PAN_HORN_SCREW_R, PAN_HORN_SCREW_R)
    )
    return a + b


def _ribbon_drop() -> Part:
    """The ribbon from the yoke slot down to its 45 deg fold, flat face to the
    axis, two layers thick at the fold, plus 0.5 mm all round (pan group)."""
    t = 2.0
    return box(
        -(RIBBON_R + t / 2),
        -(RIBBON_R - t / 2),
        -RIBBON_W / 2 - 0.5,
        RIBBON_W / 2 + 0.5,
        FLOOR + 0.5,
        PLATE_Z0,
    )


def _ribbon_wrap() -> Part:
    """Where the edgewise run can lie: against the drum from the fence's -Y end
    round to +Y, then forward along the base's +Y side (static keep-out)."""
    z0, z1 = FLOOR, FLOOR + RIBBON_W + 0.5
    r0, r1 = DRUM_R + 0.1, DRUM_R + 1.6
    wrap = ring(r0, r1, 90, 180 + GUIDE_HALF_DEG, z0, z1)
    run = box(0, BASE_X[1] + 10, r0, r1, z0, z1)
    return wrap + run


def assembly(pan_deg: float = 0.0, tilt_deg: float = 0.0) -> list[Body]:
    pan = Rot(Z=pan_deg)
    tilt = pan * Pos(0, 0, ZT) * Rot(Y=-tilt_deg) * Pos(0, 0, -ZT)  # +tilt = nose up
    c = _cached
    out = [
        Body("base", c("base", base), "base", printed=True, colour=C_STRUCT),
        Body(
            "pan_servo",
            c("pan_servo", lambda: PAN_LOC * fx.servo(MG90D)),
            "base",
            allow={"pan_horn"},
        ),
        Body(
            "pan_horn",
            pan * c("pan_horn", lambda: PAN_LOC * (Rot(Z=90) * fx.horn("double"))),
            "pan",
            allow={"pan_servo"},
        ),
        Body("yoke", pan * c("yoke", yoke), "pan", printed=True, colour="#4c535c"),
        Body(
            "tilt_servo",
            pan * c("tilt_servo", lambda: TILT_LOC * fx.servo(MG90D)),
            "pan",
            allow={"tilt_horn"},
        ),
        Body(
            "tilt_horn",
            tilt * c("tilt_horn", lambda: TILT_LOC * (Rot(Z=180) * fx.horn("single"))),
            "tilt",
            allow={"tilt_servo"},
        ),
        Body(
            "cradle", tilt * c("cradle", cradle), "tilt", printed=True, colour=C_STRUCT
        ),
        Body("head", tilt * c("head", lambda: HEAD_LOC * fx.head()), "tilt"),
    ]
    sleeve, screw = c("pivot", _pivot_hw)
    out += [
        Body("pivot_sleeve", tilt * sleeve, "tilt"),
        Body("pivot_screw", tilt * screw, "tilt"),
        Body("tilt_horn_screws", tilt * c("ths", _tilt_horn_screws), "tilt"),
        Body("pan_horn_screws", pan * c("phs", _pan_horn_screws), "pan"),
        Body("ribbon_drop", pan * c("rd", _ribbon_drop), "pan"),
        Body("ribbon_wrap", c("rw", _ribbon_wrap), "base"),
    ]
    if EYEBROWS:
        out.append(
            Body(
                "brow_bracket",
                tilt * c("brow_bracket", brow_bracket),
                "tilt",
                printed=True,
                colour="#4c535c",
            )
        )
        for i, side in enumerate((1, -1)):
            loc = BROW_LOCS[i]
            out.append(
                Body(
                    f"brow_servo_{i}",
                    tilt * c(f"bs{i}", lambda loc=loc: loc * fx.servo(SG90)),
                    "tilt",
                )
            )
            out.append(
                Body(
                    f"brow_horn_{i}",
                    tilt
                    * c(
                        f"bh{i}",
                        lambda loc=loc, side=side: (
                            loc * (Rot(Z=90 * side) * fx.horn("single", SG90))
                        ),
                    ),
                    "tilt",
                )
            )
            out.append(
                Body(
                    f"brow_{i}",
                    tilt * c(f"b{i}", lambda side=side: brow(side)),
                    "tilt",
                    printed=True,
                    colour=C_BROW,
                )
            )
    return out


def main() -> None:
    out = HERE / "output"
    out.mkdir(exist_ok=True)
    for name, part in printed_parts().items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(
            part, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1
        )
        bb = part.bounding_box().size
        print(
            f"{name:13s} {part.volume / 1000:6.2f} cm3  {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f} mm"
            f"  valid={part.is_valid}"
        )
    print(f"tilt axis z {ZT:.1f}, +Y cheek y {CHEEK_P}, -Y cheek y {CHEEK_N}")


if __name__ == "__main__":
    main()
