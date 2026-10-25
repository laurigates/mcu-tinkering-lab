"""Teuvo's Face: a 1950s robot head built round the camera as a single eye.

The camera is Teuvo's eye. It nods (tilt, MG90D) inside a fixed face frame,
the way an eyeball turns in a skull, while two SG90 eyebrows sit on a fixed
forehead above it. The whole face turns on the neck (pan, MG90D).

Why the brows are on the pan stage and not on the tilting eye: the tilt servo
then carries only the eye cradle and head 1 (about 18 g, CoM ~6 mm off the
axis), and the brow servos' 18 g and their two leads never cross the tilt
joint. It is also how a face works: brows belong to the skull, not the eye.

Printed parts (world pose at pan 0 / tilt 0; print orientation in README):

    base      neck plinth: pan servo pocket hanging from its tabs, M3 flange
    frame     the face frame: round neck ring on the pan horn, two "ear" pods;
              +Y pod holds the tilt servo, -Y pod the 683ZZ pivot bearing
    eye       the eye cradle: holds head 1 by its two M2 holes, bezel ring
              round the lens, +Y arm on the tilt horn, -Y arm on the pivot
    forehead  brow bar bolted on top of both pods, holds the two SG90s
    brow_l    left eyebrow (+Y), on the SG90 single-arm horn
    brow_r    right eyebrow (-Y)
    ear_cap   Robby-style ear plate, press-fits the pod end (print 2)
    keeper    slides into the base behind the pan pocket, carries the rear tab
              pilot; its slot lets the pan servo's lead boot drop in

World frame (common/fixtures.py): +X forward, +Y left, +Z up, mm; z 0 is the
top of the acrylic chassis plate. Pan axis = world Z through x 0, y 0. Tilt
axis = world Y through x TILT_X, z TILT_Z. Positive tilt = nose up.

Run ``python design.py`` to write output/<part>.step and .stl and print the
mass / torque budget.
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

from build123d import (
    Axis,
    Box,
    Cylinder,
    Location,
    Part,
    Plane,
    Polyline,
    Pos,
    Rot,
    export_step,
    export_stl,
    extrude,
    make_face,
    mirror,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))  # hardware/pan-tilt, for common.*

from common import fixtures as fx  # noqa: E402
from common.check import Body  # noqa: E402

# --- Caliper these before ordering (README "Measure first") ------------------
# Published TowerPro A-F are used as-is; the rest are fixture estimates. Every
# pocket, ledge and pilot below is derived from these, so a measured value goes
# here and nowhere else.
MG90D = dataclasses.replace(
    fx.MG90D,
    tab_span=31.5,  # E: sets both tab-pilot pairs
    tab_underside=19.8,  # F: sets the base ledge height and the tilt bulkhead
    tab_thickness=2.5,  # estimate: sets the tilt bulkhead's outer face
    tab_hole_inset=2.3,  # estimate: tab hole centre from the tab end
    shaft_from_end=6.0,  # estimate: shaft axis from the case's near end
)
SG90 = dataclasses.replace(
    fx.SG90,
    tab_span=32.3,
    tab_underside=15.9,  # estimate: sets the forehead plate's inner face
    shaft_from_end=6.0,  # estimate
)
# Case clearance, each side. Width is generous: MJF holes print undersize and
# a servo 0.25 mm over its published width must still drop in. Length is held
# to 0.4 because the tab pilots sit just past the case ends (the pilot web is
# tab_span / 2 - tab_hole_inset - PILOT_M2 / 2 - length / 2 - CASE_CLR_L, 1.0
# mm on the MG90D), and the tab screws, not the pocket, locate the servo there.
CASE_CLR_W = 0.5
CASE_CLR_L = 0.4
PILOT_M2 = 1.5  # self-tapping pilot, servo tab screws (2 mm screws in PA12/resin)
PILOT_M3 = 2.5  # self-tapping M3 pivot screw in PA12
CLEAR_M2 = 2.3
CLEAR_M3 = 3.4

# --- Pan stage ---------------------------------------------------------------
BASE_FLOOR = 2.0
F_SLACK = 1.0  # pocket floor below the servo's base: hangs from its tabs
LEDGE_Z = BASE_FLOOR + F_SLACK + MG90D.tab_underside  # tab underside, 22.8
PAN_TOP = LEDGE_Z + (MG90D.height - MG90D.tab_underside)  # pan case top, 31.5
HORN_ARM_Z0 = 0.5 + 4.0 - 1.6  # fixtures.horn: arm underside above the case
FLOOR_U = PAN_TOP + HORN_ARM_Z0  # frame floor underside = horn arm underside
FLOOR_POCKET_TOP = PAN_TOP + 4.5 + 0.2
FLOOR_T = FLOOR_POCKET_TOP + 2.8  # frame floor top, 39.0
NECK_R = 26.0
MOUNT_DX, MOUNT_DY = 42.0, 25.0  # M3 chassis pattern, centred on the servo body
MOUNT_CX = MG90D.shaft_from_end - MG90D.length / 2  # -5.3

# --- Tilt stage --------------------------------------------------------------
TILT_X = 0.0  # tilt axis above the pan axis
TILT_Z = 68.0  # set by the J2 keep-out's sweep at tilt +45 clearing the floor
HEAD_AXIS_X = 6.0  # tilt axis in head-1 coordinates: 6 mm in front of the
HEAD_AXIS_Z = fx.XIAO_CENTRE_YZ[1]  # board's back face, at the lens height
ARM_IN = fx.HEAD_SIZE / 2 + 0.5  # eye cradle arm inner face, |y|
ARM_OUT = 19.5  # +Y arm outer face = tilt horn arm underside
TILT_CASE_Y = ARM_OUT + HORN_ARM_Z0  # tilt servo case top, 22.4
BULK_Y0 = TILT_CASE_Y + 1.6  # +Y bulkhead inner face, 24.0
BULK_Y1 = TILT_CASE_Y + (MG90D.height - MG90D.tab_underside) - MG90D.tab_thickness
POD_Y1 = 56.0  # pod open ends, |y|: 2.5 mm past an MG90D F of 20.8 + the cap rim
POD_HX = MG90D.width / 2 + CASE_CLR_W  # pod interior half-width
WALL = 2.0
POD_TOP_T = 6.0  # pod roof: the forehead's M2 screws thread 5.9 mm into it
POD_Z_TOP_IN = (
    MG90D.shaft_from_end - MG90D.length / 2 + MG90D.tab_span / 2 + 0.55
)  # interior top (rel axis), clears the upper tab
POD_TOP = POD_Z_TOP_IN + POD_TOP_T  # pod roof top, rel axis (17.0)
CHEEK_Y0 = -24.0  # -Y cheek inner face = bearing inner face
BEARING = (3.0, 7.0, 3.0)  # 683ZZ: id, od, width
LIP_BORE = 6.0  # holds the outer race, passes the screw head and shim
SPACER_L = 4.0  # bearing inner race to the eye's -Y boss
BOSS_Y = CHEEK_Y0 + SPACER_L  # -20.0

# --- Eye (tilt-frame coordinates: origin on the tilt axis) -------------------
HX = -HEAD_AXIS_X  # board back face, x
HZ = -HEAD_AXIS_Z  # board bottom edge, z
BOARD_TOP = HZ + fx.HEAD_SIZE
STACK_FRONT = HX + fx.HEAD_THICK + fx.XIAO_STACK[0]  # lens plane, 21.6
BEZEL_X0 = STACK_FRONT + 0.4
BEZEL_X1 = BEZEL_X0 + 2.5
BEZEL_R = 12.5
APERTURE_R = 8.5  # estimate: lens position unmeasured (fixture says ~stack centre)
LENS_YZ = (fx.XIAO_CENTRE_YZ[0], 0.0)
PLATE_X0 = HX - 3.0

# --- Forehead (tilt-frame coordinates, fixed to the pan stage) ---------------
BROW_SHAFT_Y = 14.0  # |y| of the brow servo shafts
FH_X = 22.0  # SG90 case tops (shaft side), x
FH_Z = 37.5  # SG90 shaft axis height
FH_HZ = 9.0  # forehead half height
FH_Y = 40.5  # forehead half width
LEG_Y0 = 38.5  # cavity half width (SG90 lead boots end at 37)
POST_Y = 35.25  # |y| of the four posts and M2 screws onto the pod tops
POST_X = 4.5
BROW_DROP = 19.0  # brow centre below its shaft (horn arm hangs straight down)
# Brow travel from neutral, positive = inner ends down (stern). Negative swings
# the inner ends up and toward each other; they touch at -18 (brow_sweep()).
BROW_LIMITS = (-15, 45)

COLOURS = {
    "base": "#2e3033",  # MJF PA12S-HP, dyed black
    "frame": "#8d9093",  # MJF PA12-HP, natural grey: a Salora radio cabinet
    "eye": "#1d1d1f",  # black: no flare round the lens
    "forehead": "#e8e0cc",
    "brow_l": "#c3c7cc",  # SLM AlSi10Mg, polished: chrome brows
    "brow_r": "#c3c7cc",
    "ear_cap": "#c9dde3",  # 8001 translucent resin: Robby's lit ears
    "keeper": "#2e3033",  # MJF PA12S-HP, black, with the base
}


def box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0
    )


def cyl_y(x, z, y0, y1, r) -> Part:
    """Cylinder along Y from y0 to y1."""
    return Location((x, (y0 + y1) / 2, z), (90, 0, 0)) * Cylinder(r, abs(y1 - y0))


def cyl_x(y, z, x0, x1, r) -> Part:
    return Location(((x0 + x1) / 2, y, z), (0, 90, 0)) * Cylinder(r, abs(x1 - x0))


def cyl_z(x, y, z0, z1, r) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


def T() -> Location:
    """Tilt-frame origin in the pan frame at pan 0."""
    return Pos(TILT_X, 0, TILT_Z)


# --- Servo placements ---------------------------------------------------------


def tilt_servo_loc() -> Location:
    # shaft points -Y (inward), case long axis up: servo X -> +Z, Y -> -X
    return Location(
        Plane(origin=(0, TILT_CASE_Y, 0), x_dir=(0, 0, 1), z_dir=(0, -1, 0))
    )


def brow_servo_loc(side: int) -> Location:
    # shaft forward (+X), shaft end of the case inward; side +1 = +Y (left)
    return Location(
        Plane(
            origin=(FH_X, side * BROW_SHAFT_Y, FH_Z),
            x_dir=(0, -side, 0),
            z_dir=(1, 0, 0),
        )
    )


def _tab_pilots(spec) -> list[float]:
    """Tab hole positions along the servo's X axis."""
    return [p[0] for p in fx.servo_tab_holes(spec)]


# --- Printed parts (pan frame; tilt-frame parts are placed via T()) -----------


def make_base() -> Part:
    s = MG90D
    x0 = s.shaft_from_end - s.length - CASE_CLR_L
    x1 = s.shaft_from_end + CASE_CLR_L
    tab0 = MOUNT_CX - s.tab_span / 2
    tab1 = MOUNT_CX + s.tab_span / 2
    plinth = box(tab0 - 2.55, tab1 + 2.45, -8.7, 8.7, 0, LEDGE_Z)
    plinth = plinth.fillet(3.0, plinth.edges().filter_by(Axis.Z))
    fl_x0, fl_x1 = MOUNT_CX - MOUNT_DX / 2 - 5, MOUNT_CX + MOUNT_DX / 2 + 5
    flange = box(fl_x0, fl_x1, -MOUNT_DY / 2 - 5, MOUNT_DY / 2 + 5, 0, 3.0)
    flange = flange.fillet(4.0, flange.edges().filter_by(Axis.Z))
    b = plinth + flange
    b -= box(
        x0, x1, -s.width / 2 - CASE_CLR_W, s.width / 2 + CASE_CLR_W, BASE_FLOOR, 99
    )
    # lead window: the boot leaves the -X end low, the JR plug threads through
    b -= box(tab0 - 4, x0 + 0.1, -4.5, 4.5, 6.5, 13.0)
    # keeper slot: open at the top and the rear so the lead boot can drop
    # straight down behind the pocket while the servo is lowered in. The rear
    # tab's pilot is in the keeper that refills the slot afterwards (B3).
    b -= _keeper_slot(KEEPER_CLR)
    for x in _tab_pilots(s)[1:]:  # front tab only; the rear one is in the keeper
        b -= cyl_z(x, 0, LEDGE_Z - 7, LEDGE_Z + 1, PILOT_M2 / 2)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b -= cyl_z(
                MOUNT_CX + sx * MOUNT_DX / 2, sy * MOUNT_DY / 2, -1, 4, CLEAR_M3 / 2
            )
    # ribbon / lead tie bar slot at the back of the flange
    b -= box(fl_x0 + 2.0, fl_x0 + 4.0, -6, 6, -1, 4)
    return b


KEEPER_HY = 2.4  # keeper slot half width (lead boot 4 mm wide)
KEEPER_CLR = 0.2  # keeper in its slot, each side
KEEPER_Z0 = 13.2  # keeper underside: above the lead window


def _keeper_slot(grow: float) -> Part:
    """The rear-tab keeper's envelope (grow=0) or its slot (grow=clearance).

    A vertical block behind the pan pocket plus a rib each side that runs in a
    groove in the plinth walls, so the keeper slides in from the rear and
    cannot lift; the rear tab screw then pins it along X.
    """
    s = MG90D
    x0 = s.shaft_from_end - s.length - CASE_CLR_L  # pocket rear end
    rear = MOUNT_CX - s.tab_span / 2 - 2.55  # plinth rear face
    if grow > 0:  # the slot: through the rear face, down into the lead window
        xa, xb, c = rear - 1, x0 + 0.01, 0.0
        k = box(xa, xb, -KEEPER_HY, KEEPER_HY, 12.9, LEDGE_Z + 1)
    else:
        xa, xb, c = rear, x0, KEEPER_CLR
        k = box(xa, xb, -KEEPER_HY + c, KEEPER_HY - c, KEEPER_Z0, LEDGE_Z)
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (KEEPER_HY - 0.5), sy * (KEEPER_HY + 1.0 - c)))
        k += box(xa, xb, y0, y1, 15.0 + c, 17.0 - c)
    return k


def make_keeper() -> Part:
    """Rear-tab keeper (world pose): slides into the base from behind."""
    k = _keeper_slot(0.0)
    x = _tab_pilots(MG90D)[0]
    k -= cyl_z(x, 0, LEDGE_Z - 7, LEDGE_Z + 1, PILOT_M2 / 2)
    return k


# Lead route under the frame (pan-frame coordinates, |y| mirrored per side)
LEAD_SLOT_Y = (29.0, 50.0)  # rear-wall slot, |y|, from the pod floor up 8 mm
LEAD_HOLE_X = (3.0, 7.0)  # pod-floor hole: passes a JR plug (2.6 x 7.6)
LEAD_HOLE_Y = (29.0, 46.5)
LEAD_CHAN_X = (4.0, 9.6)  # underside channel, open below and to the front
LEAD_CHAN_TOP = 37.4  # channel roof: 1.6 mm under the floor top
LEAD_DROP_Y = (9.0, 11.2)  # where the bundle leaves the frame, |y|
LEAD_TIE_X = ((4.0, 5.1), (8.45, 9.55))  # two 1.1 x 3.4 slots for a 2.5 x 1 mm tie
LEAD_TIE_Y = (11.8, 15.2)
LEAD_BUNDLE_X = (5.2, 8.4)  # two 3-way servo leads side by side, 3.2 x 2.2
RIBBON_SLOT_X = (-22.5, -20.0)


def make_frame() -> Part:
    """The face frame in the pan frame at pan 0 (world pose)."""
    a = TILT_Z
    z_floor_rel = FLOOR_T - a
    pod_z0, pod_z1 = FLOOR_T, a + POD_TOP
    hx = POD_HX + WALL
    f = cyl_z(0, 0, FLOOR_U, FLOOR_T, NECK_R)
    # the cross plate under the pods is full neck depth: it carries the lead
    # channels on its underside
    f += box(TILT_X - hx, TILT_X + hx, -POD_Y1, POD_Y1, FLOOR_U, FLOOR_T)
    # pods, both sides (+Y from the bulkhead, -Y from the cheek)
    f += box(TILT_X - hx, TILT_X + hx, BULK_Y0, POD_Y1, pod_z0, pod_z1)
    f += box(TILT_X - hx, TILT_X + hx, -POD_Y1, CHEEK_Y0, pod_z0, pod_z1)
    f = f.fillet(
        2.0,
        f.edges()
        .filter_by(Axis.Y)
        .filter_by(lambda e: e.center().Z > FLOOR_T + 1 and abs(e.center().Y) > 30),
    )
    zi0, zi1 = FLOOR_T, a + POD_Z_TOP_IN
    xi0, xi1 = TILT_X - POD_HX, TILT_X + POD_HX
    f -= box(xi0, xi1, BULK_Y1, POD_Y1 + 1, zi0, zi1)
    f -= box(xi0, xi1, -POD_Y1 - 1, CHEEK_Y0 - 5.0, zi0, zi1)
    # tilt servo window through the bulkhead (case above the tabs)
    s = MG90D
    sx0, sx1 = s.shaft_from_end - s.length, s.shaft_from_end
    f -= box(
        TILT_X - s.width / 2 - CASE_CLR_W,
        TILT_X + s.width / 2 + CASE_CLR_W,
        BULK_Y0 - 1,
        BULK_Y1 + 0.01,
        a + sx0 - CASE_CLR_L,
        a + sx1 + CASE_CLR_L,
    )
    for zt in _tab_pilots(s):
        f -= cyl_y(TILT_X, a + zt, BULK_Y0 - 0.5, BULK_Y1 + 0.5, PILOT_M2 / 2)
    # pivot: 683ZZ pressed in from the inside against an outer lip
    _bid, bod, bw = BEARING
    f -= cyl_y(TILT_X, a, CHEEK_Y0 + 0.5, CHEEK_Y0 - bw, bod / 2)
    f -= cyl_y(TILT_X, a, CHEEK_Y0 - bw + 0.01, CHEEK_Y0 - 5.5, LIP_BORE / 2)
    # Lead route (M3): each brow lead comes down the back of its pod, in
    # through a low slot in the pod's rear wall, across the pod floor and down
    # a hole in it; the tilt lead drops from its boot to the same hole. Under
    # the plate a channel takes the bundle inward into the neck's front
    # quadrant pocket, where a cable tie through two slots holds it at r ~12
    # and it drops to the base. Nothing outboard of the tie moves relative to
    # the frame, so only the drop below the tie flexes with pan.
    for sy in (-1, 1):

        def ys(a_, b_):
            return sorted((sy * a_, sy * b_))

        # flat-faced land where the channel crosses the neck ring, so the cut
        # meets a plane at 90 degrees instead of leaving a knife edge
        f += box(8.0, 12.0, *ys(22.0, 28.0), FLOOR_U, FLOOR_T)
        f -= box(
            TILT_X - hx - 1,
            xi0 + 0.01,
            *ys(LEAD_SLOT_Y[0], LEAD_SLOT_Y[1]),
            zi0 - 0.01,
            zi0 + 8.0,
        )
        f -= box(*LEAD_HOLE_X, *ys(*LEAD_HOLE_Y), FLOOR_U - 1, FLOOR_T + 0.01)
        f -= box(
            *LEAD_CHAN_X,
            *ys(LEAD_DROP_Y[0], LEAD_HOLE_Y[1] + 2.5),
            FLOOR_U - 1,
            LEAD_CHAN_TOP,
        )
        for tx in LEAD_TIE_X:
            f -= box(*tx, *ys(*LEAD_TIE_Y), LEAD_CHAN_TOP - 0.1, FLOOR_T + 1)
    # forehead screw pilots straight down into the 6 mm pod roof (no bosses:
    # they blocked the tilt servo's way in, B2); 5.9 mm of thread for M2 x 20
    for sy in (-1, 1):
        for bx in (-POST_X, POST_X):
            f -= cyl_z(
                TILT_X + bx, sy * POST_Y, zi1 - 0.5, pod_z1 + 1, PILOT_M2 / 2 + 0.05
            )
    # pan horn pocket (cross horn) + hub, horn screw holes, centre access
    clr = 0.3
    pocket = box(
        -16 - clr, 16 + clr, -2.5 - clr, 2.5 + clr, FLOOR_U - 1, FLOOR_POCKET_TOP
    )
    pocket += box(
        -2.5 - clr, 2.5 + clr, -16 - clr, 16 + clr, FLOOR_U - 1, FLOOR_POCKET_TOP
    )
    pocket += cyl_z(0, 0, FLOOR_U - 1, FLOOR_POCKET_TOP, 3.5 + clr)
    f -= pocket
    f -= cyl_z(0, 0, FLOOR_U, FLOOR_T + 1, 3.0)
    # lighten the neck ring from below: four quadrant pockets between the arms
    rib = 2.8 + 2.0
    for qx in (-1, 1):
        for qy in (-1, 1):
            q = box(
                *sorted((qx * rib, qx * 30)),
                *sorted((qy * rib, qy * 30)),
                FLOOR_U - 1,
                FLOOR_T - 2.0,
            )
            f -= q & cyl_z(0, 0, FLOOR_U - 2, FLOOR_T, NECK_R - 2.0)
    for px, py in ((12, 0), (-12, 0), (0, 12), (0, -12)):
        f -= cyl_z(px, py, FLOOR_U, FLOOR_T + 1, 1.1)
    # ribbon gate at the back of the neck ring: the 14-way ribbon threads
    # under this bridge (16 x 2.2 slot), so a tug lands on the frame, not J2,
    # then turns straight down through the floor slot under the bridge. It
    # leaves the turning stage at r ~21 inside the neck instead of dragging
    # over the ring's back edge (M4).
    gate = box(-23.5, -19.0, -9.5, 9.5, FLOOR_T - 0.01, FLOOR_T + 4.2)
    gate -= box(-24.0, -18.5, -8.0, 8.0, FLOOR_T - 0.02, FLOOR_T + 2.2)
    f += gate
    f -= box(*RIBBON_SLOT_X, -8.25, 8.25, FLOOR_U - 1, FLOOR_T + 0.5)
    del z_floor_rel
    return f


def make_eye() -> Part:
    """The eye cradle in tilt-frame coordinates."""
    e = box(PLATE_X0, HX, -ARM_IN, ARM_IN, HZ + 8.0, BOARD_TOP)  # back plate
    for sy in (-1, 1):
        y0, y1 = sorted((sy * ARM_IN, sy * ARM_OUT))
        e += box(PLATE_X0, 16.0, y0, y1, -11.0, BOARD_TOP)
        e += box(15.0, BEZEL_X1, y0, y1, -4.6, 4.6)
    # bottom bar under the bezel only: it starts in front of the lens plane, so
    # head 1 slides up into the cradle from below past it (B1)
    e += box(STACK_FRONT + 0.1, BEZEL_X1, -ARM_OUT, ARM_OUT, -13.0, -11.0)
    e += box(10.0, BEZEL_X1, -ARM_OUT, -ARM_IN + 0.01, -13.0, -10.0)
    e += box(10.0, BEZEL_X1, ARM_IN - 0.01, ARM_OUT, -13.0, -10.0)
    e += cyl_x(LENS_YZ[0], 0, BEZEL_X0, BEZEL_X1, BEZEL_R)
    e += cyl_x(LENS_YZ[0], 0, BEZEL_X1 - 0.01, BEZEL_X1 + 1.2, APERTURE_R + 2.0)
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (BEZEL_R - 1.5), sy * ARM_IN))
        e += box(BEZEL_X0, BEZEL_X1, y0, y1, -3.0, 3.0)
    e -= cyl_x(LENS_YZ[0], 0, BEZEL_X0 - 1, BEZEL_X1 + 2, APERTURE_R)
    # +Y arm: double horn tongue and pocket (arms along X), horn screw bosses
    e += box(-21.8, 16.0, ARM_IN, ARM_OUT, -4.6, 4.6)
    for hx_ in (-14.0, 14.0):
        e += cyl_y(hx_, 0, ARM_IN + 0.01, 12.5, 2.5)
    pocket = box(-20.3, 20.3, ARM_OUT - 1.8, ARM_OUT + 1, -3.05, 3.05)
    pocket += cyl_y(0, 0, ARM_OUT - 1.8, ARM_OUT + 1, 3.8)
    e -= pocket
    for hx_ in (-14.0, 14.0):
        e -= cyl_y(hx_, 0, ARM_OUT - 1.7, 12.0, PILOT_M2 / 2)
    # -Y arm: pivot boss, M3 self-tap pilot
    e += cyl_y(0, 0, -ARM_IN - 0.01, BOSS_Y, 4.0)
    e -= cyl_y(0, 0, BOSS_Y - 1, -ARM_IN + 1, PILOT_M3 / 2)
    # head-1 M2 screw pilots in the back plate
    for y, z in fx.HEAD_HOLES_YZ:
        e -= cyl_x(y, HZ + z, PLATE_X0 - 1, HX + 0.5, 0.8)
    # J2 ribbon plug keep-out, with 0.5 mm
    xd, yd, zd = fx.J2_KEEPOUT
    jy, jz = fx.J2_CENTRE_YZ
    e -= box(
        HX - xd - 0.5,
        HX + 0.01,
        jy - yd / 2 - 0.5,
        jy + yd / 2 + 0.5,
        HZ + jz - zd / 2 - 0.5,
        HZ + jz + zd / 2 + 0.5,
    )
    # lighten the arms
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (ARM_IN - 1), sy * (ARM_OUT + 1)))
        e -= box(-4.0, 9.0, y0, y1, 5.5, 10.0)
    return e


def make_forehead() -> Part:
    """Brow bar in tilt-frame coordinates (fixed to the frame)."""
    s = SG90
    xf = FH_X - 0.4
    xr = FH_X - s.height - 2.0
    x_tab = FH_X - (s.height - s.tab_underside) + s.tab_thickness  # tab top face
    z0, z1 = FH_Z - FH_HZ, FH_Z + FH_HZ
    pod_top = POD_TOP
    fb = box(xr, xf, -FH_Y, FH_Y, z0, z1)
    for sy in (-1, 1):
        for bx in (-POST_X, POST_X):
            fb += cyl_z(bx, sy * POST_Y, pod_top, z0 + 0.01, 3.7)
    fb = fb.fillet(
        1.2,
        fb.edges().filter_by(Axis.Y).filter_by(lambda e: e.center().X > xf - 0.1),
    )
    # open-back cavity for both servos and their leads
    hz = s.width / 2 + CASE_CLR_W
    fb -= box(xr - 1, x_tab, -LEG_Y0, LEG_Y0, FH_Z - hz, FH_Z + hz)
    # windows for the case above the tabs; blind tab pilots
    for side in (-1, 1):
        loc = brow_servo_loc(side)
        c0 = s.shaft_from_end - s.length - CASE_CLR_L
        c1 = s.shaft_from_end + CASE_CLR_L
        ya, yb = sorted(
            ((loc * Pos(c0, 0, 0)).position.Y, (loc * Pos(c1, 0, 0)).position.Y)
        )
        fb -= box(x_tab - 0.01, xf + 1, ya, yb, FH_Z - hz, FH_Z + hz)
        for tx in _tab_pilots(s):
            yy = (loc * Pos(tx, 0, 0)).position.Y
            fb -= cyl_x(yy, FH_Z, x_tab - 0.5, x_tab + 6.0, PILOT_M2 / 2)
    # four M2 x 20 screws down the posts into the 6 mm pod roofs; the heads
    # sit on the cavity floor (31.1 above the axis), the tips end at 11.1,
    # 0.65 above the tilt servo's upper tab: 5.9 mm of thread each (M1)
    for sy in (-1, 1):
        for bx in (-POST_X, POST_X):
            fb -= cyl_z(bx, sy * POST_Y, FH_Z - 2.0, z1 + 1, 2.2)
            fb -= cyl_z(bx, sy * POST_Y, pod_top - 1, FH_Z - 1.99, CLEAR_M2 / 2)
    return fb


def make_brow(side: int) -> Part:
    """Brow in its servo frame (horn arm hangs along servo +Y*side), angle 0.

    Servo X points inward (toward the face's centre line); d is the distance
    below the brow servo's shaft. A plain tapered bar, outer end lifted: the
    whole of Teuvo's range of feeling is in how it is tilted.
    """
    sg = side
    z0, z1 = HORN_ARM_Z0, HORN_ARM_Z0 + 4.0
    yc = BROW_DROP
    pts = [  # (X, d): top edge inner -> outer, then bottom edge outer -> inner
        (8.0, yc - 1.0),
        (0.0, yc - 2.6),
        (-10.0, yc - 3.6),
        (-13.0, yc - 3.6),
        (-13.0, yc + 1.9),
        (-10.0, yc + 1.9),
        (0.0, yc + 3.3),
        (8.0, yc + 2.5),
    ]
    face = make_face(Polyline(*[(x, sg * d) for x, d in pts], close=True))
    br = Pos(0, 0, z0) * extrude(face, z1 - z0, dir=(0, 0, 1))
    # short stalk over the horn arm's tip, pocket for the arm, one horn screw
    br += box(-4.6, 4.6, *sorted((sg * 11.5, sg * yc)), z0, z1)
    br -= box(-3.05, 3.05, *sorted((sg * 11.0, sg * 20.3)), z0 - 0.1, z0 + 1.8)
    br -= Pos(0, sg * 14.0, 0) * Cylinder(CLEAR_M2 / 2, 20)
    return br


def make_ear_cap() -> Part:
    """+Y ear cap in pan-frame coordinates (the -Y one is the same part turned)."""
    a = TILT_Z
    hx = POD_HX + WALL
    z0, z1 = FLOOR_T, a + POD_TOP
    c = box(TILT_X - hx, TILT_X + hx, POD_Y1, POD_Y1 + 2.5, z0 + 0.3, z1)
    c = c.fillet(1.5, c.edges().filter_by(Axis.Y))
    zi0, zi1 = FLOOR_T + 0.15, a + POD_Z_TOP_IN - 0.15
    hi = POD_HX - 0.15
    rim = box(TILT_X - hi, TILT_X + hi, POD_Y1 - 2.5, POD_Y1 + 0.01, zi0, zi1)
    rim -= box(
        TILT_X - hi + 1.6,
        TILT_X + hi - 1.6,
        POD_Y1 - 3,
        POD_Y1 + 0.02,
        zi0 + 1.6,
        zi1 - 1.6,
    )
    c += rim
    # Robby ear: a raised disc with a ring
    zc = (z0 + z1) / 2
    c += cyl_y(TILT_X, zc, POD_Y1 + 2.49, POD_Y1 + 4.5, 7.5)
    c -= cyl_y(TILT_X, zc, POD_Y1 + 3.5, POD_Y1 + 5, 5.0)
    c += cyl_y(TILT_X, zc, POD_Y1 + 3.0, POD_Y1 + 5.5, 3.0)
    return c


# --- Hardware stand-ins (tilt-frame coordinates) -------------------------------


def pivot_hardware() -> tuple[Part, Part]:
    """(bearing, rotating stack: shim + spacer + M3x14 socket screw)."""
    bid, bod, bw = BEARING
    bearing = cyl_y(0, 0, CHEEK_Y0, CHEEK_Y0 - bw, bod / 2) - cyl_y(
        0, 0, CHEEK_Y0 + 1, CHEEK_Y0 - bw - 1, bid / 2
    )
    spacer = cyl_y(0, 0, BOSS_Y, CHEEK_Y0, 2.25) - cyl_y(
        0, 0, BOSS_Y + 1, CHEEK_Y0 - 1, 1.55
    )
    y_head = CHEEK_Y0 - bw - 0.5
    shim = cyl_y(0, 0, CHEEK_Y0 - bw, y_head, 2.5)
    head = cyl_y(0, 0, y_head, y_head - 3.0, 2.75)
    shank = cyl_y(0, 0, y_head, y_head + 14.0, 1.5)
    return bearing, spacer + shim + head + shank


# --- Lead and ribbon stand-ins (pan frame, world pose at pan 0) ----------------
LEAD_T = 1.1  # one flat 3-way servo lead, 3.2 x 1.1


def lead_paths(side: int) -> tuple[Part, Part | None]:
    """(brow lead, tilt lead or None) on one side, as 3.2 x 1.1 flat runs.

    Modelled from each boot to where the bundle leaves the frame, so the gate
    proves the route is clear of every moving part at every pose. Below the
    tie the bundle hangs free to the base; that part bends and is not a body.
    """

    def ys(a_, b_):
        return sorted((side * a_, side * b_))

    fz = FLOOR_T + 0.1
    a = TILT_Z
    cav = a + FH_Z - (SG90.width / 2 + CASE_CLR_W) + 0.1  # forehead cavity floor
    boot_z0 = a + FH_Z - 2.0  # SG90 boot underside
    bx0, bx1 = LEAD_BUNDLE_X
    brow = (
        box(-1.6, 1.6, *ys(31.2, 32.3), cav, boot_z0 + 0.1)  # down from the boot
        + box(-10.4, 1.6, *ys(29.6, 32.8), cav, cav + LEAD_T)  # back across the cavity
        + box(-10.4, -9.3, *ys(29.6, 32.8), fz, cav + LEAD_T)  # down the pod's back
        + box(-10.4, bx0 + LEAD_T, *ys(29.6, 32.8), fz, fz + LEAD_T)  # pod floor
    )
    # down the floor hole, then inward under the plate as a 2-lead bundle
    bundle = (
        box(bx0, bx0 + LEAD_T, *ys(29.6, 45.6), 35.0, fz + LEAD_T)
        + box(bx0, bx1, *ys(LEAD_DROP_Y[0], 45.6), 35.0, 37.2)
        + box(bx0, bx1, *ys(*LEAD_DROP_Y), FLOOR_U - 1.0, 37.2)
    )
    brow += bundle
    if side < 0:
        return brow, None
    # tilt lead: MG90D boot (pod interior, low) straight down, along the floor
    s = MG90D
    boot_zb = a + s.shaft_from_end - s.length  # case's lower end, world z
    boot_y0 = TILT_CASE_Y + s.height - 5.0 - 4.0  # boot's inner face, y
    tilt = box(-1.6, 1.6, boot_y0 + 0.5, boot_y0 + 1.6, fz, boot_zb - 6.0 + 0.1)
    tilt += box(-1.6, bx0 + LEAD_T, boot_y0 + 0.5, boot_y0 + 3.7, fz, fz + LEAD_T)
    return brow, tilt


def lead_tie(side: int) -> Part:
    """The 2.5 mm cable tie's strap where it crosses the floor top."""
    ya, yb = sorted((side * (LEAD_TIE_Y[0] + 0.2), side * (LEAD_TIE_Y[1] - 0.2)))
    return box(
        LEAD_TIE_X[0][0], LEAD_TIE_X[1][1], ya, yb, FLOOR_T + 0.01, FLOOR_T + 1.1
    )


def ribbon_path() -> Part:
    """The 14-way ribbon from under the gate bridge down through the floor slot
    to just above the pan servo's rear tab (z 25.3). The tilt loop above it and
    the pan service loop below it bend and are not bodies."""
    x0 = RIBBON_SLOT_X[0] + 0.3
    return box(x0, -18.6, -7.0, 7.0, FLOOR_T + 0.4, FLOOR_T + 1.4) + box(
        x0, x0 + 1.1, -7.0, 7.0, LEDGE_Z + MG90D.tab_thickness + 0.7, FLOOR_T + 1.4
    )


# --- Assembly -----------------------------------------------------------------

_CACHE: dict[str, Part] = {}


def _part(name: str) -> Part:
    if name not in _CACHE:
        _CACHE[name] = {
            "base": make_base,
            "frame": make_frame,
            "eye": lambda: T() * make_eye(),
            "forehead": lambda: T() * make_forehead(),
            "brow_l": lambda: T() * brow_servo_loc(1) * make_brow(1),
            "brow_r": lambda: T() * brow_servo_loc(-1) * make_brow(-1),
            "ear_cap": make_ear_cap,
            "keeper": make_keeper,
        }[name]()
    return _CACHE[name]


def printed_parts() -> dict[str, Part]:
    """World pose at pan 0 / tilt 0, brows at neutral. ear_cap: print 2."""
    return {n: _part(n) for n in COLOURS}


def assembly(pan_deg: float, tilt_deg: float, brow_deg: float = 0.0) -> list[Body]:
    pan = Rot(0, 0, pan_deg)
    tilt = T() * Rot(0, -tilt_deg, 0) * T().inverse()
    tl = T() * tilt_servo_loc()
    bodies = [
        Body("base", _part("base"), "base", printed=True, colour=COLOURS["base"]),
        Body(
            "keeper", _part("keeper"), "keeper", printed=True, colour=COLOURS["keeper"]
        ),
        Body(
            "pan_servo",
            Pos(0, 0, PAN_TOP) * fx.servo(MG90D),
            "pan_servo",
            allow={"pan_horn"},
        ),
        Body("pan_horn", pan * Pos(0, 0, PAN_TOP) * fx.horn("cross"), "pan"),
        Body(
            "frame", pan * _part("frame"), "pan", printed=True, colour=COLOURS["frame"]
        ),
        Body(
            "forehead",
            pan * _part("forehead"),
            "pan",
            printed=True,
            colour=COLOURS["forehead"],
        ),
        Body(
            "ear_cap_l",
            pan * _part("ear_cap"),
            "pan",
            printed=True,
            colour=COLOURS["ear_cap"],
        ),
        Body(
            "ear_cap_r",
            pan * mirror(_part("ear_cap"), Plane.XZ),
            "pan",
            printed=True,
            colour=COLOURS["ear_cap"],
        ),
        Body(
            "tilt_servo",
            pan * tl * fx.servo(MG90D),
            "tilt_servo",
            allow={"tilt_horn"},
        ),
    ]
    # the lead and ribbon runs ride the pan stage but are their own group, so
    # the gate also proves they sit in their channels clear of the frame
    for side, tag in ((1, "l"), (-1, "r")):
        brow_run, tilt_run = lead_paths(side)
        bodies.append(
            Body(
                f"lead_brow_{tag}", pan * brow_run, "leads", allow={f"brow_servo_{tag}"}
            )
        )
        if tilt_run is not None:
            bodies.append(
                Body("lead_tilt", pan * tilt_run, "leads", allow={"tilt_servo"})
            )
        bodies.append(Body(f"lead_tie_{tag}", pan * lead_tie(side), "leads"))
    bodies.append(Body("ribbon", pan * ribbon_path(), "leads"))
    bearing, stack = pivot_hardware()
    bodies.append(Body("bearing", pan * T() * bearing, "pan", allow={"pivot_stack"}))
    bodies += [
        Body(
            "tilt_horn",
            pan * tilt * tl * Rot(0, 0, 90) * fx.horn("double"),
            "tilt",
        ),
        Body(
            "eye",
            pan * tilt * _part("eye"),
            "tilt",
            printed=True,
            colour=COLOURS["eye"],
        ),
        Body("pivot_stack", pan * tilt * T() * stack, "tilt"),
        Body(
            "head",
            pan * tilt * T() * Pos(HX, 0, HZ) * fx.head(),
            "head",
        ),
    ]
    for side, tag in ((1, "l"), (-1, "r")):
        bl = T() * brow_servo_loc(side)
        rot = Rot(0, 0, brow_deg * side)
        bodies += [
            Body(
                f"brow_servo_{tag}",
                pan * bl * fx.servo(SG90),
                f"brow_servo_{tag}",
                allow={f"brow_horn_{tag}"},
            ),
            Body(
                f"brow_horn_{tag}",
                pan * bl * rot * Rot(0, 0, 90 * side) * fx.horn("single", SG90),
                f"brow_{tag}",
            ),
            Body(
                f"brow_{tag}",
                pan * bl * rot * make_brow(side),
                f"brow_{tag}",
                printed=True,
                colour=COLOURS[f"brow_{tag}"],
            ),
        ]
    return bodies


# --- Mass and torque budget ---------------------------------------------------

DENSITY = {"PA12": 1.01, "resin": 1.15}  # g/cm3 (MJF PA12-HP; 9600/black resin)
HEAD_MASS_G = 13.0  # estimate: 2.6 g board + XIAO S3 Sense ~7 g + headers + IDC plug
MG90D_STALL_KGCM = 2.1  # at 4.8 V (TowerPro)
RIBBON_TORQUE_KGCM = 0.03  # estimate: 14-way ribbon bend, 10 cm loop


def torque_budget() -> dict:
    eye = _part("eye")
    m_eye = eye.volume / 1000 * DENSITY["PA12"]
    c_eye = eye.center()
    head = T() * Pos(HX, 0, HZ) * fx.head(screws=False)
    c_head = head.center()
    m = m_eye + HEAD_MASS_G + 1.0  # + horn and screws
    cx = (m_eye * c_eye.X + HEAD_MASS_G * c_head.X) / m
    cz = (m_eye * c_eye.Z + HEAD_MASS_G * c_head.Z) / m
    lever_mm = math.hypot(cx - TILT_X, cz - TILT_Z)  # worst case: lever horizontal
    t = m * lever_mm / 10 / 1000 + RIBBON_TORQUE_KGCM  # kg*cm
    return {
        "tilt_mass_g": round(m, 1),
        "com_offset_mm": round(lever_mm, 1),
        "tilt_torque_kgcm": round(t, 3),
        "stall_margin": round(MG90D_STALL_KGCM / t, 0),
    }


def brow_sweep(angles=(-18, -15, 0, 30, 45, 60), tilts=(-45, 0, 45)) -> dict:
    """The brows' own travel, which common.check does not sweep."""
    from common.check import clashes

    return {
        f"brow {b} tilt {t}": c
        for b in angles
        for t in tilts
        if (c := clashes(assembly(0, t, b)))
    }


def main() -> None:
    out = HERE / "output"
    out.mkdir(exist_ok=True)
    for name, part in printed_parts().items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"))
        print(f"{name}: {part.volume / 1000:.2f} cm3, valid={part.is_valid}")
    print(torque_budget())
    if "--brows" in sys.argv:
        print(brow_sweep())


if __name__ == "__main__":
    main()
