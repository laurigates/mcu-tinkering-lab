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
RIBBON_SLOT = (
    -29.0,
    -26.0,
    fx.J2_CENTRE_YZ[0] - 8.5,
    fx.J2_CENTRE_YZ[0] + 8.5,
)  # x0 x1 y0 y1

# Base
BASE_X = (-23.0, 12.4)
BASE_Y = 9.0
FLANGE_Y = 21.0
MOUNT_HOLES = [(x, y) for x in (-15.3, 4.7) for y in (-17.0, 17.0)]  # M3, 20 x 34 mm

# Pivot hardware (stand-ins): M3 x 14 socket head, 4/3.1 brass tube sleeve 7.0 long
SLEEVE_L = 7.0
SCREW_L = 14.0

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
PAN_LOC = Pos(0, 0, PAN_TOP)
TILT_LOC = Pos(0, Y_CASE, ZT) * Rot(X=90)  # shaft points -Y, body outboard (+Y)
BROW_LOCS = [
    HEAD_LOC * Pos(BROW_CASE_X, s * BROW_Y, BROW_SHAFT_Z) * Rot(Y=90) for s in (1, -1)
]


def _servo_body_x(spec):
    """Case x range in the servo frame."""
    return spec.shaft_from_end - spec.length, spec.shaft_from_end


# --- printed parts (world pose, pan 0 tilt 0) --------------------------------
def base() -> Part:
    s = MG90D
    flange = box(BASE_X[0], BASE_X[1], -FLANGE_Y, FLANGE_Y, 0, FLOOR)
    block = box(BASE_X[0], BASE_X[1], -BASE_Y, BASE_Y, 0, PAN_TAB_Z)
    p = flange + block
    bx0, bx1 = _servo_body_x(s)
    # The pocket is open on the -Y side: the servo slides in sideways (+Y) with
    # its tabs gliding over the rim, so the lead's root never has to pass a
    # wall. Both tab pilots sit in the full-width end walls.
    p -= box(bx0 - FIT, bx1 + FIT, -BASE_Y - 1, s.width / 2 + FIT, FLOOR, PAN_TAB_Z + 1)
    # lead exit: a slot through the -X end wall, open to the -Y side and down
    # through the floor, so the lead drops through a chassis hole or runs back
    p -= box(
        BASE_X[0] - 1, bx0, -BASE_Y - 1, 2.75, FLOOR - 0.01, FLOOR + PAN_GAP + 10.0
    )
    p -= box(BASE_X[0] - 1, bx0, -2.75, 2.75, -1, FLOOR + 1)
    for x, _, _ in fx.servo_tab_holes(s):
        p -= cyl_z(x, 0, PILOT_SERVO / 2, PAN_TAB_Z - 7.0, PAN_TAB_Z + 1)
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
    for y in (-14.0, 14.0):  # horn arm screws, driven up through the horn's holes
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
    # ribbon slot + the bar behind it takes a cable tie (strain relief)
    x0, x1, y0, y1 = RIBBON_SLOT
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
    # brow bracket screws, blind from behind, 0.6 mm short of the board
    for y in (-11.25, 11.25):
        p -= cyl_x(y, 16.0, PILOT_M2 / 2, -BACK_T - 1, -1.2)
    # tilt horn (single arm pointing rearward) in a pocket on the +Y arm face
    # horn frame: servo z axis = -Y; its flat top lies HORN_ARM_Z0+1.6 inboard of the case
    horn_cut = _horn_pocket("single", 180, 2.9 - 0.2, 4.5 + 0.2)
    p -= Pos(ax, Y_ARM_P + HORN_ARM_Z0, az) * Rot(X=90) * horn_cut
    p -= cyl_y(ax, az, 2.8, ARM_IN - 1, Y_ARM_P + 1)  # horn screw head + spline end
    for r in (10.0, 16.0):  # horn arm screws (from the horn side)
        p -= cyl_y(ax - r, az, PILOT_SERVO / 2, ARM_IN - 1, Y_ARM_P + 1)
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
    p = box(x0, x1, -15.3, 15.3, 10.0, zt)
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
    for y in (-11.25, 11.25):
        p -= cyl_x(y, 16.0, CLEAR_M2 / 2, x0 - 1, x1 + 1)
        p -= cyl_x(
            y, 16.0, 2.1, x0 - 1, x0 + 1.0
        )  # 1 mm counterbore: M2x8 stops short of the board
    # lighten the middle between the screws
    p -= box(x0 - 1, x1 + 1, -5.0, 5.0, 13.0, 26.0)
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
