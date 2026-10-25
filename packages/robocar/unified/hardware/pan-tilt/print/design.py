"""The Gimbal, print revision: the pan-tilt Lauri orders for robocar-unified head 1.

This is the tournament winner (``entries/gimbal/design.py``, kept unchanged as
the record) with the judges' criticisms fixed and three ideas grafted from the
runners-up. What changed, by part (README.md has the reasons):

- tower: the 6705ZZ's outer race is now clamped by a screwed **bearing cap**
  (new part), not held by crush ribs alone, so lifting the robot by the head
  cannot pull the bearing out. Tower Ø40.8 to carry the cap's three M2 holes.
- keeper: crush ribs on the cross-slot walls take out the horn backlash
  (was about +-1 degree) without stopping the axial float. A second keeper
  with the slot turned 9 degrees halves the pan-zero error of the 18-degree
  spline step.
- yoke: ribs beside the tilt servo stiffen the 3 mm upright in its weak
  direction; the upright reaches z +20 with two tapped holes for the optional
  eyebrow pod; flange holes give a driver to the cap screws; the cable shelf
  and gutter moved out to clear the cap.
- outrigger: a rib on its outboard face; the pin is printed OVERSIZE (Ø8.25)
  and sanded or reamed down to a slide fit in the MR128ZZ; a shoulder on the
  pin's root limits how far the cradle can walk. The shoulder gap is printed at
  1.0 mm and set on assembly with a feeler, through foot holes slotted +-1 mm
  in Y, so the tolerance stack (tab thickness, horn height, MJF) is taken up at
  the bench. It also reaches z +20 for the right brow pod (-Y).
- cradle: the MR128ZZ pocket is Ø12.4 with three crush ribs (0.15 mm per side
  on the 12.0 OD), like the pan seat, instead of a bonded slip fit. The ribbon
  tie slots moved to a tab below the plate, so the strap crosses the plate's
  front face below the head board, 2 mm clear of both insert holes.
- tower: the four M3 chassis holes have captive-nut pockets in bosses, and the
  screws go in from under the acrylic, so the tower can be bolted down at any
  time (the yoke, cap and gutter block a key from above).
- optional eyebrows (graft: brows on the fixed pan-stage forehead, so they pan
  but do not tilt): two SG90 pods bolted to the upright and the outrigger, two
  brows on SG90 single horns. The pod mount screws sit clear of the SG90's tab
  and are in their own clash group, so the gate checks them against the servo.
- the ribbon is tied to the cradle's back, so its free tilt span starts near
  the tilt axis, and the ribbon post rises above the shelf so the pan span no
  longer wraps round the cap and gutter. Both free spans are in the clash gate
  as taut proxies at every pose (the tilt span as a 14 mm strip, the pan span
  only as its Ø1.5 centre line).
- the bearing cap's screws are reached through arc slots in the yoke's
  flange, so they line up at any spline step.
- coupon.stl: a bench coupon of the two thinnest webs (an MG90D tab hole beside
  its case window, and the cradle's insert hole beside a tie slot).

Original design notes follow.

The Gimbal: a nodal pan-tilt for robocar-unified head 1 on two MG90D servos.

Design intent, in order:

1. The tilt axis runs through the camera's entrance pupil, so tilting turns the
   view without translating it (no parallax for the ADR-025 planner).
2. The pan axis sits under the centre of mass of everything that pans, in X.
   In Y the CoM sits about 8 mm toward the tilt-servo side (+Y; ``main()``
   prints it); that moment goes into the 6705ZZ, not the pan servo's shaft.
3. Height and swept volume are minimised around those two constraints: the
   yoke and outrigger are cut by the head's own tilt sweep (the head is
   subtracted from them at every 2 degrees of tilt), and the lens height is
   computed from that sweep, not guessed.
4. The pan thrust and the side-mounted tilt servo's overturning moment go into
   a thin-section 6705ZZ ball bearing around the pan horn, not the servo shaft.
   The yoke is clamped to the bearing's inner race by a printed keeper, and the
   horn's trimmed arms float axially in the keeper's cross slot, so the pan
   servo carries torque only. The tilt side opposite the servo runs in an
   MR128ZZ, so the tilt servo's output shaft is not a cantilever.
5. The IDC ribbon and the servo leads have a modelled path: a shelf and a
   gutter on the yoke, a post on the tower, tie slots on all three, and
   ``ribbon_spans()`` sizes the two free spans.

Head 1 is mounted upside down (rotated 180 degrees about the camera axis,
``HEAD_FLIPPED``): the J2 ribbon plug then sits behind the board's top edge
and swings up and back when the head looks up, instead of down into the pan
stage. That alone lowers the lens about 12 mm. The image is turned with the
sensor's own flips (the head is an OV3660); the firmware already has them
(``CAMERA_VFLIP_DEFAULT`` / ``CAMERA_HMIRROR_DEFAULT``, README "Firmware notes").
With ``HEAD_FLIPPED = False`` everything below re-derives (taller, ~83 mm).

Frames. Geometry is built in the LENS frame: origin on the tilt axis at the
entrance pupil, +X forward, +Y left, +Z up. ``WORLD`` puts the pan axis at
x = y = 0 and the chassis plate's top at z = 0 (common/fixtures.py world frame:
+X forward, +Z up, mm). ``printed_parts()`` returns each part in its WORLD pose
at pan 0, tilt 0 (``coupon`` has its own flat pose and is not in the
assembly). All parts are MJF PA12, which needs no supports, so print
orientation is free; the README lists the preferred one per part.

Run ``python design.py`` to write <part>.step and .stl next to this file
(print/), the files that are ordered.
"""

from __future__ import annotations

import functools
import math
import sys
from pathlib import Path

import numpy as np
from build123d import (
    Align,
    Box,
    Cylinder,
    Location,
    Part,
    Plane,
    Pos,
    Rot,
    Vector,
    export_step,
    export_stl,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # hardware/pan-tilt, for common.*

from common import fixtures as fx  # noqa: E402
from common.check import Body  # noqa: E402

MIN, CEN, MAX = Align.MIN, Align.CENTER, Align.MAX

# --------------------------------------------------------------------------
# Parameters. The first block is what Lauri must caliper before ordering.
# --------------------------------------------------------------------------
SERVO = fx.MG90D  # tab span E, below-tab depth F, shaft offset: see README
# Entrance pupil behind the front face of the XIAO stack keep-out. Estimate:
# the OV3660 module's M8-class lens has its pupil ~2 mm behind its front glass.
PUPIL_BEHIND_STACK_FRONT = 2.0
HEAD_FLIPPED = True

FIT = 0.3  # case side clearance: a 12.6 window takes TowerPro's 12.2 headline
FIT_END = 0.35  # case end clearance: a 23.3 window takes TowerPro's 22.8 headline
FIT_TAB = 0.6  # tab-span cavity: 31.5 + 1.2 = 32.7 takes even the SG90's 32.3
GAP = 0.5  # running clearance between parts that move relative to each other
SWEEP_MARGIN = 1.0  # the head/cradle proxy is grown by this before cutting
TILT_CUT = 46.0  # the yoke clears the head over +-this (check sweeps +-45)
TILT_STEP = 2.0

# Pan bearing: thin-section 6705ZZ (25 x 32 x 4) around the cross horn.
PB_ID, PB_OD, PB_W = 25.0, 32.0, 4.0
PB_INNER_RACE_OD = 27.6  # land on the inner race stays inside this
# Tilt idler bearing: MR128ZZ (8 x 12 x 3.5) in the cradle's -Y cheek.
TB_ID, TB_OD, TB_W = 8.0, 12.0, 3.5
# Printed fits on precision bearings are not trusted to MJF's +-0.3. The pin is
# printed OVERSIZE and sanded or reamed down to a slide fit (sanding only makes
# a pin smaller, so it must start above 8.0 even at MJF -0.3), and is NOT
# bonded: the inner race must slide on it so the cradle's axial place is set by
# the tilt spline alone (the shoulder only limits the walk). The pocket takes
# the outer race on three crush ribs, like the pan seat.
TB_PIN_D = 8.25  # sand / ream to a slide fit in the 8.0 bore; no compound
TB_POCKET_D = 12.4  # clearance bore; the three ribs below locate the 12.0 OD
TB_POCKET_CRUSH = 0.15  # rib interference on the OD, per side
TB_POCKET_RIB_A = (90.0, 210.0, 330.0)  # degrees in the XZ plane (90 = up)
TB_SHOULDER_R = (
    4.45  # pin-root shoulder Ø8.9: inner race only; must clear the shield ID
)
TB_SHOULDER_GAP = 1.0  # as printed; set to ~0.3 with a feeler via the slotted feet
TB_SHOULDER_H = 0.6  # the shoulder stands this proud of the boss face
OUTRIGGER_SLOT = 1.0  # foot holes slotted +-this in Y to set the shoulder gap

WALL = 2.0
# Servo tab holes and the keeper screws are TAPPED M2 (drill Ø1.6, tap M2x0.4)
# and take M2 machine screws. Tapping avoids a self-tapper's forming load, but
# a loaded 60-degree thread still pushes the web outward (about tan 30 = 0.58 of
# the axial load), and the web beside the MG90D case window is ~0.8 mm after
# tapping: tighten those four screws by feel (README "Walls"; coupon.stl).
PILOT = 1.6
SCREW_CORE = 1.55  # stand-in: an M2 machine screw's minor diameter
INSERT_HOLE = 3.2  # M2 x 3 heat-set insert (OD 3.5 knurl) in PA12
PAN_SERVO_LIFT = 2.0  # pan servo base above the chassis plate (F + 2 mm still clears)
CHASSIS_FLANGE_T = 3.0
CHASSIS_PLATE_T = 3.0  # acrylic 2WD plate, a stand-in in assembly()
# Head screws: the driver reaches the M2 button heads from the front along
# their axes. A Ø3 hex-driver shaft (1.3 mm hex) plus 0.3 mm all round.
HEAD_DRIVER_D = 3.6
M3_HOLES = [(-20.0, 16.0), (-20.0, -16.0), (15.0, 16.0), (15.0, -16.0)]  # world xy
# Each M3 hole has a boss with a captive hex-nut pocket open at the top; the
# M3 x 10 goes in from under the acrylic. A key from above cannot reach the
# front pair past the cap and flange (r 21.8), nor the +Y rear one past the gutter.
M3_BOSS_R, M3_BOSS_TOP = 4.5, 6.0
M3_NUT_AF, M3_NUT_DEPTH = 5.8, 2.6  # pocket for an ISO 4032 M3 nut (AF 5.5 x 2.4)

# Panned centre of mass behind the lens; derived by panned_com() below and
# frozen here so the geometry is deterministic. Re-run `python design.py`
# after a change: it prints the CoM and warns if this drifts > 0.5 mm.
PAN_AXIS_BEHIND_LENS = 8.5

# Optional SG90 eyebrows (separate parts). True puts them in the assembly so
# the clash gate checks them; the required parts carry their mounting holes
# either way.
BROWS = True

# Mass estimates (g) for the CoM. Stand-ins are keep-outs, so their centroids
# are only approximate centres of mass.
DENSITY_PA12 = 1.01e-3  # g/mm3 (HP 3D HR PA12, solid)
M_SERVO = 13.0  # TowerPro: 13 g
M_HEAD_BOARD = 4.0  # 29.5 x 29.5 x 1.6 FR-4 (2.6 g) + headers and parts
M_XIAO = 7.0  # XIAO ESP32-S3 Sense with camera board, in female headers
M_PLUG = 1.5  # IDC plug + the first few cm of ribbon
M_HORN, M_TB, M_INSERTS = 1.0, 1.2, 0.6

# --------------------------------------------------------------------------
# The head in the lens frame
# --------------------------------------------------------------------------
STACK_FRONT_X = fx.HEAD_THICK + fx.XIAO_STACK[0]
LENS_IN_HEAD = Vector(
    STACK_FRONT_X - PUPIL_BEHIND_STACK_FRONT, fx.XIAO_CENTRE_YZ[0], fx.XIAO_CENTRE_YZ[1]
)
HEAD_LOC = (Rot(180, 0, 0) if HEAD_FLIPPED else Location()) * Pos(-LENS_IN_HEAD)


def hp(x: float, y: float, z: float) -> Vector:
    """A point given in the fixture's head frame, in the lens frame."""
    return (HEAD_LOC * Pos(x, y, z)).position


BOARD_BACK_X = hp(0, 0, 0).X  # -25.6
BOARD_FRONT_X = hp(fx.HEAD_THICK, 0, 0).X
_board_ys = sorted([hp(0, -fx.HEAD_SIZE / 2, 0).Y, hp(0, fx.HEAD_SIZE / 2, 0).Y])
_board_zs = sorted([hp(0, 0, 0).Z, hp(0, 0, fx.HEAD_SIZE).Z])
BOARD_Y0, BOARD_Y1 = _board_ys
BOARD_Z0, BOARD_Z1 = _board_zs
HOLES = [hp(0, y, z) for (y, z) in fx.HEAD_HOLES_YZ]  # lens frame, on back face
STACK_HALF_Y = fx.XIAO_STACK[1] / 2  # stack is centred on the lens in y

# --------------------------------------------------------------------------
# Tilt servo (+Y side): shaft on the tilt axis pointing -Y, body hanging down.
# servo frame x -> world +Z, y -> world -X, z -> world -Y.
# --------------------------------------------------------------------------
CHEEK_IN = STACK_HALF_Y + 0.6  # inner faces of the cradle cheeks, |y|
HORN_CHEEK_T = 5.0  # +Y inner cheek: 1.8 horn pocket + 3.2 floor
HORN_POCKET = 1.8
TILT_SERVO_TOP_Y = CHEEK_IN + HORN_CHEEK_T - HORN_POCKET + 4.5  # horn arm top on floor
TILT_SERVO_LOC = Location(
    Plane(origin=(0, TILT_SERVO_TOP_Y, 0), x_dir=(0, 0, 1), z_dir=(0, -1, 0))
)
TAB_IN_Y = TILT_SERVO_TOP_Y + (SERVO.height - SERVO.tab_underside) - SERVO.tab_thickness
UPRIGHT_T = 3.0
UPRIGHT_Y0, UPRIGHT_Y1 = TAB_IN_Y - UPRIGHT_T, TAB_IN_Y

# Idler side (-Y)
IDLER_CHEEK_T = TB_W + WALL  # bearing pocket + 2 mm lip
IDLER_FACE_Y = -(CHEEK_IN + IDLER_CHEEK_T)  # outer face of the -Y inner cheek
OUTRIGGER_BOSS_Y = IDLER_FACE_Y - GAP
OUTRIGGER_PLATE_Y1 = BOARD_Y0 - 3.0 - 1.25  # 1.25 outside the cradle's -Y outer cheek
OUTRIGGER_PLATE_Y0 = OUTRIGGER_PLATE_Y1 - 3.0

# --------------------------------------------------------------------------
# Pan stage heights (world z). Pan servo shaft on the pan axis, body back.
# --------------------------------------------------------------------------
# The pan coupling is torque-only. A stock cross horn, its arms cut to
# HORN_TRIM_R, sits on the pan spline under its own centre screw. The arm
# stubs float in a cross slot through the KEEPER, a printed ring screwed under
# the yoke's spigot that also clamps the 6705ZZ's inner race against the
# yoke's land. Yoke + bearing + keeper are one bench subassembly, so the
# bearing alone locates the yoke and the servo shaft carries only torque.
ZS = PAN_SERVO_LIFT + SERVO.height  # pan servo case top
Z_CHAMBER = ZS  # chamber floor flush with the case top (inside r 14.3)
CASE_TOP_CLEAR = ZS + 0.5  # the case pocket's roof, outside the chamber
HORN_ARM_Z0 = ZS + 2.9  # fixture horn arm 2.9..4.5 above the case
Z_HORN_TOP = ZS + 4.5
KEY_FLOAT = 1.0  # axial float of the horn in the keeper, each way
HORN_TRIM_R = 10.0  # cross horn arms cut to this radius (flush cutters)
KEY_FIT = 0.3  # arm stub to slot wall, per side; the ribs below take it up
KEY_RIB_R, KEY_RIB_AT, KEY_CRUSH = (
    0.6,
    7.0,
    0.1,
)  # rib radius, radius from axis, interference
Z_KEEPER0 = HORN_ARM_Z0 - KEY_FLOAT
Z_SEAT0 = Z_HORN_TOP + KEY_FLOAT  # keeper top = inner race bottom
Z_TOWER_TOP = Z_SEAT0 + PB_W  # bearing top flush with the tower top
SPIGOT_SHORT = 0.2  # spigot is shorter than the race, so the keeper clamps it
# Bearing cap: a printed ring screwed to the tower top that clamps the 6705ZZ's
# outer race down onto the seat floor. Its bore clears the yoke's land, and it
# rides captive on the yoke between the flange and the bearing until it is
# screwed down. M2 x 6 button heads sit in counterbores, flush with its top.
CAP_T = 2.6
CAP_BORE_R = 15.2  # bears on the outer race face from r 15.2 to 16.0
CAP_SCREW_R = 18.45
CAP_SCREW_A = (60.0, 180.0, 300.0)  # degrees about +Z; ribs sit between
CAP_CB_D, CAP_CB_DEPTH = 4.0, 1.5  # counterbore for an ISO 7380 M2 head (3.5 x 1.3)
Z_CAP_TOP = Z_TOWER_TOP + CAP_T
Z_LAND_TOP = Z_CAP_TOP + GAP
FLANGE_T = 4.0
Z_FLANGE_TOP = Z_LAND_TOP + FLANGE_T
TOWER_R = 20.4  # was 18.8: carries the cap's tapped M2 holes at r 18.45
CAP_R = TOWER_R + 1.4  # the cap overhangs the tower so its counterbores keep 1.3 mm
FLANGE_R = CAP_R  # the yoke flange covers the cap; its driver holes keep 2 mm outside
CAP_DRIVER_D = (
    2.6  # flange slots: a 1.3 hex key or a 2.5 driver reaches pre-placed screws
)
CAP_SLOT_DEG = 10.0  # the slots run +-10 degrees: the horn's spline step is 18
CHAMBER_R = PB_INNER_RACE_OD / 2 + 0.5  # 14.3: clear of the turning inner race
KEEPER_R = PB_INNER_RACE_OD / 2 - 0.5  # 13.3: bears on the inner race only
KEEPER_SCREW_R = 10.3  # 4 x M2 x 8, tapped into the spigot, at 45 degrees
SEAT_R = PB_OD / 2 + 0.2  # seat bore, with three crush ribs
RIB_A = (0.0, 120.0, 240.0)  # crush-rib angles, clear of the cap screws
CRUSH = 0.15  # ribs' interference with the 6705ZZ's OD, per side

# Cable path (world, about the pan axis). The ribbon is tied to a shelf on the
# yoke's rear edge, hangs in a pan service loop and is tied again to a post on
# the tower's rear. The tilt servo's lead runs in a gutter round the yoke's
# rim from the +Y arm to the same shelf.
# Everything that hangs below the flange stays 0.5 outside the cap (r 21.8).
SHELF_R0, SHELF_R1, SHELF_HW = CAP_R + 0.5, CAP_R + 7.2, 10.5
SHELF_T = 4.0  # shelf top flush with the flange top, so the pan span leaves above it
# The post stands 0.5 outside the shelf's swept corners (r 30.8) and rises
# above the shelf, so the pan span runs from the shelf's rear edge over the
# post's top, clear of the cap, gutter and flange it used to wrap round.
POST_X0, POST_X1, POST_HW = -35.5, -31.5, 10.5
RIBBON_W, RIBBON_T = 14.0, 1.2  # 14-way 1.0 mm pitch IDC ribbon
POST_TOP = Z_LAND_TOP + SHELF_T + RIBBON_T + 3.0
GUTTER_A0, GUTTER_A1 = 100.0, 148.0  # degrees about +Z, from +X: ends before the shelf
GUTTER_R = tuple(
    CAP_R + 0.5 + d for d in (0.0, 1.5, 5.0, 6.5)
)  # walls 1.5, channel 3.5


def _box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(MIN, MIN, MIN))


def _cyl_y(r, y0, y1, x=0.0, z=0.0) -> Part:
    """Cylinder along Y from y0 to y1."""
    return Pos(x, y0, z) * (
        Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=(CEN, CEN, MIN))
    )


def _cyl_z(r, z0, z1, x=0.0, y=0.0) -> Part:
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(CEN, CEN, MIN))


def bored_horn(kind: str) -> Part:
    """The fixture horn with its hub bored to the spline it sits on."""
    spline = Cylinder(
        SERVO.spline_d / 2, SERVO.overall - SERVO.height, align=(CEN, CEN, MIN)
    )
    return fx.horn(kind, SERVO) - spline


# --------------------------------------------------------------------------
# Cradle (tilts with the head): back plate behind the board's hole edge, two
# outer cheeks past the board edges, bridges, two inner cheeks up to the axis.
# --------------------------------------------------------------------------
PLATE_T = 3.6
HOLE_Z = HOLES[0].Z
PLATE_Z0 = HOLE_Z - 3.55 if HOLE_Z < 0 else HOLE_Z - 3.65
PLATE_Z1 = HOLE_Z + 3.65 if HOLE_Z < 0 else HOLE_Z + 3.55
OUTER_T = 2.5
BRIDGE_X0, BRIDGE_X1 = (
    BOARD_FRONT_X + 2.0,
    BOARD_FRONT_X + 4.5,
)  # 0.7 past the screw heads
# The bridges and inner cheeks stop short of the head screws' driver line
# (HEAD_DRIVER_D about each hole axis, + 0.35), so a driver reaches both
# button heads from the front along their axes.
_DRV_EDGE = abs(HOLE_Z) - HEAD_DRIVER_D / 2 - 0.35  # 7.5 for the flipped head
CHEEK_TAB_Z = (-_DRV_EDGE, 6.0) if HOLE_Z < 0 else (-6.0, _DRV_EDGE)


# The ribbon runs from the plug down the back of the head, lying on the
# cradle's back, and is tied through two slots in the cradle plate. A strap
# looped through two slots crosses the face opposite the ribbon: the plate's
# front face, where the head board sits flush. So the plate carries a TIE BLOCK
# that rises behind the board above the insert holes (the space between the
# board's back and the ribbon is empty there: J2 is at the top), and the strap
# lies in a groove TIE_GROOVE deep in its front face, below the board's back
# face. The groove and the slots keep >= 4 mm to both insert holes. That
# stretch moves with the head, so it is part of what tilts (it is in the head's
# keep-outs below and cuts the yoke like the rest); the free tilt span starts
# where the ribbon leaves the plate's bottom edge. Lens frame.
RIBBON_Y = 7.0  # the J2 plug's centre, lens y (the flipped head)
TIE_SLOT_Z0, TIE_SLOT_Z1 = -4.0, -1.0  # slots and groove: 3 mm for a 2.5 mm tie
TIE_SLOT_W = 1.5
TIE_BLOCK_TOP = TIE_SLOT_Z1 + 1.5
TIE_GROOVE = 1.2  # a 2.5 mm tie is about 1.0 thick
RIBBON_ON_HEAD = (
    BOARD_BACK_X - 3.6 - 1.2,
    BOARD_BACK_X - 3.6,
    RIBBON_Y - 7.0,
    RIBBON_Y + 7.0,
    -13.8,
    9.3,
)
RIBBON_ANCHOR = (
    RIBBON_ON_HEAD[0] + 0.6,
    RIBBON_Y,
    -14.4,
)  # free span starts below the plate
RIBBON_TIE_Y = (
    RIBBON_Y - 7.0 - 1.5,
    RIBBON_Y + 7.0 + 1.25,
)  # slot centres in the cradle plate


def head_proxy_boxes(m: float = 0.0) -> list[tuple]:
    """The fixture head's keep-outs (board + screw heads, stack, J2) as
    lens-frame boxes (x0, x1, y0, y1, z0, z1), grown by m."""
    out = []
    xs, ys, zs = fx.XIAO_STACK
    xd, yd, zd = fx.J2_KEEPOUT
    cy, cz = fx.XIAO_CENTRE_YZ
    jy, jz = fx.J2_CENTRE_YZ
    for lo, hi in [
        (
            (0, -fx.HEAD_SIZE / 2, 0),
            (fx.HEAD_THICK + 1.3, fx.HEAD_SIZE / 2, fx.HEAD_SIZE),
        ),
        (
            (fx.HEAD_THICK, cy - ys / 2, cz - zs / 2),
            (fx.HEAD_THICK + xs, cy + ys / 2, cz + zs / 2),
        ),
        ((-xd, jy - yd / 2, jz - zd / 2), (0, jy + yd / 2, jz + zd / 2)),
    ]:
        a, b = hp(*lo), hp(*hi)
        out.append(
            (
                min(a.X, b.X) - m,
                max(a.X, b.X) + m,
                min(a.Y, b.Y) - m,
                max(a.Y, b.Y) + m,
                min(a.Z, b.Z) - m,
                max(a.Z, b.Z) + m,
            )
        )
    x0, x1, y0, y1, z0, z1 = RIBBON_ON_HEAD
    out.append((x0 - m, x1 + m, y0 - m, y1 + m, z0 - m, z1 + m))
    return out


def _box_points(x0, x1, y0, y1, z0, z1, n: int = 24) -> np.ndarray:
    ny = math.ceil((y1 - y0) / 0.4) + 1
    g = np.meshgrid(
        np.linspace(x0, x1, n), np.linspace(y0, y1, ny), np.linspace(z0, z1, n)
    )
    return np.stack(g, -1).reshape(-1, 3)


@functools.lru_cache(maxsize=1)
def _tilting_points() -> np.ndarray:
    """Points filling/covering everything that tilts: the head keep-outs (grid)
    and the cradle (surface samples), lens frame, at tilt 0."""
    import trimesh

    pts = [_box_points(*b) for b in head_proxy_boxes(0.0)]
    verts, tris = build_cradle().tessellate(0.05)
    mesh = trimesh.Trimesh(np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(tris))
    samp, _ = trimesh.sample.sample_surface_even(mesh, 40000, seed=1)
    pts += [samp, mesh.vertices]
    return np.concatenate(pts)


def _rotate_y(p: np.ndarray, deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)  # build123d Rot(0, deg, 0): x' = c x + s z
    return np.stack([c * p[:, 0] + s * p[:, 2], p[:, 1], -s * p[:, 0] + c * p[:, 2]], 1)


def _swept_poly(xz: np.ndarray, a_max: float, m: float, bin_deg: float = 2.0):
    """Polygon (x, z) enclosing a point set swept about the tilt axis through
    +-a_max degrees, grown radially by m. Per polar bin the max (and min) radius
    any rotated point reaches, widened over neighbouring bins and for the chord
    sag, so the polygon is a superset of the true sweep."""
    rot = [
        _rotate_y(np.stack([xz[:, 0], np.zeros(len(xz)), xz[:, 1]], 1), a)
        for a in np.arange(-a_max, a_max + 1e-6, 0.5)
    ]
    p = np.concatenate(rot)
    r = np.hypot(p[:, 0], p[:, 2])
    phi = np.degrees(np.arctan2(p[:, 2], p[:, 0]))
    nb = round(360 / bin_deg)
    idx = np.clip(((phi + 180.0) / bin_deg).astype(int), 0, nb - 1)
    rmax = np.full(nb, -np.inf)
    rmin = np.full(nb, np.inf)
    np.maximum.at(rmax, idx, r)
    np.minimum.at(rmin, idx, r)
    np.isfinite(rmax)
    rmax_n, rmin_n = rmax.copy(), rmin.copy()
    for k in (-2, -1, 1, 2):
        rmax_n = np.maximum(rmax_n, np.roll(rmax, k))
        rmin_n = np.minimum(rmin_n, np.roll(rmin, k))
    rmax_n = rmax_n / math.cos(math.radians(bin_deg)) + m
    rmin_n = np.maximum(rmin_n - m, 0.0)
    centres = np.radians(-180.0 + (np.arange(nb) + 0.5) * bin_deg)
    ring = np.isfinite(rmax_n)
    if ring.all():  # wraps the axis: star polygon on the outer radius
        return [(rr * math.cos(c), rr * math.sin(c)) for rr, c in zip(rmax_n, centres)]
    # an arc: rotate the bin order so the gap is at the ends, fill inner gaps
    start = int(np.nonzero(~ring)[0][0])
    order = [(start + i) % nb for i in range(nb)]
    used = [i for i in order if ring[i]]
    i0, i1 = order.index(used[0]), order.index(used[-1])
    span = order[i0 : i1 + 1]
    rmx = np.array([rmax_n[i] for i in span])
    rmn = np.array([rmin_n[i] if np.isfinite(rmin_n[i]) else 0.0 for i in span])
    for j in range(len(span)):
        if not np.isfinite(rmx[j]):
            rmx[j] = np.nanmax(rmx[np.isfinite(rmx)])
            rmn[j] = 0.0
    rmn = np.maximum(rmn, 0.3)
    outer = [
        (rmx[j] * math.cos(centres[i]), rmx[j] * math.sin(centres[i]))
        for j, i in enumerate(span)
    ]
    inner = [
        (rmn[j] * math.cos(centres[i]), rmn[j] * math.sin(centres[i]))
        for j, i in reversed(list(enumerate(span)))
    ]
    return outer + inner


SLAB = 1.5  # y slab width for the swept prisms


@functools.lru_cache(maxsize=4)
def _sweep_tools_cached(m: float) -> tuple:
    from build123d import Polyline, extrude, make_face

    p = _tilting_points()
    y_lo, y_hi = p[:, 1].min(), p[:, 1].max()
    tools = []
    y = y_lo
    while y < y_hi + 1e-6:
        y0, y1 = y, min(y + SLAB, y_hi)
        sel = p[(p[:, 1] >= y0 - 1e-6) & (p[:, 1] <= y1 + 1e-6)]
        if len(sel):
            poly = _swept_poly(sel[:, [0, 2]], TILT_CUT, m)
            face = make_face(Polyline(*[(x, y1 + m, z) for (x, z) in poly], close=True))
            tools.append(extrude(face, amount=(y1 - y0) + 2 * m, dir=(0, -1, 0)))
        y += SLAB
    return tuple(tools)


def sweep_tools(m: float = SWEEP_MARGIN) -> list[Part]:
    """Everything that tilts, swept over +-TILT_CUT and grown by m, as y-slab prisms."""
    return list(_sweep_tools_cached(m))


def yoke_footprint(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Plan-view mask of the yoke's flange and arms (lens frame)."""
    disc = np.hypot(x + PAN_AXIS_BEHIND_LENS, y) < FLANGE_R
    arms = (np.abs(x) <= 9.0) & (y >= FOOT_Y0 + 0.3) & (y <= UPRIGHT_Y1)
    return disc | arms


@functools.lru_cache(maxsize=1)
def lens_height() -> float:
    """World z of the lens, so that everything that tilts, at any tilt within
    +-45 degrees, stays SWEEP_MARGIN above the yoke's flange and arms (the yoke
    is never cut through its flange) and the stationary tower top."""
    p = _tilting_points()
    sw = np.concatenate([_rotate_y(p, a) for a in np.arange(-45.0, 45.01, 1.0)])
    over = yoke_footprint(sw[:, 0], sw[:, 1])
    # tools are grown by SWEEP_MARGIN plus bin/chord slack; keep 1 mm more
    need = Z_FLANGE_TOP + SWEEP_MARGIN + 1.0 - sw[over, 2].min()
    return round(float(need), 2)


def W(z_world: float) -> float:
    """World z to lens-frame z."""
    return z_world - lens_height()


def build_cradle() -> Part:
    x_back = BOARD_BACK_X
    y_out_p = BOARD_Y1 + 0.5
    y_out_n = BOARD_Y0 - 0.5
    z0, z1 = PLATE_Z0, PLATE_Z1
    plate = _box(x_back - PLATE_T, x_back, y_out_n - OUTER_T, y_out_p + OUTER_T, z0, z1)
    zc0, zc1 = min(z0, CHEEK_TAB_Z[0]), max(z1, CHEEK_TAB_Z[1])
    c = plate
    for (ya, yb), (yi0, yi1) in [
        ((y_out_p, y_out_p + OUTER_T), (CHEEK_IN, CHEEK_IN + HORN_CHEEK_T)),
        ((y_out_n - OUTER_T, y_out_n), (-(CHEEK_IN + IDLER_CHEEK_T), -CHEEK_IN)),
    ]:
        outer = _box(x_back - PLATE_T, BRIDGE_X1, ya, yb, zc0, zc1)
        # bridges span the cheeks' z only: clear of the head screws' driver line
        bridge = _box(BRIDGE_X0, BRIDGE_X1, min(ya, yi0), max(yb, yi1), *CHEEK_TAB_Z)
        # the pivot boss is trimmed to the cheek's z, clear of the driver line
        inner = _box(BRIDGE_X0, 0, yi0, yi1, *CHEEK_TAB_Z) + (
            _cyl_y(8.0, yi0, yi1) & _box(-9, 9, yi0, yi1, CHEEK_TAB_Z[0], 9.0)
        )
        c = c + outer + bridge + inner
    # the bridges must not reach into the stack: trim to |y| >= CHEEK_IN
    c = c - _box(BRIDGE_X0 - 1, 9, -CHEEK_IN, CHEEK_IN, -20, 20)
    # M2 heat-set inserts behind the board holes (through, so no skin)
    for h in HOLES:
        c = c - Pos(x_back - PLATE_T - 1, h.Y, h.Z) * (
            Rot(0, 90, 0)
            * Cylinder(INSERT_HOLE / 2, PLATE_T + 2, align=(CEN, CEN, MIN))
        )
    # +Y: single-arm horn pocket (arm toward -X), hub, centre-screw access
    y_face = CHEEK_IN + HORN_CHEEK_T
    y_floor = y_face - HORN_POCKET
    c = c - _box(-20.4, 0, y_floor, y_face + 1, -3.05, 3.05)
    c = c - _cyl_y(3.75, y_floor, y_face + 1)
    c = c - _cyl_y(2.8, CHEEK_IN - 1, y_floor + 0.1)
    for d in HORN_SCREW_D:
        c = c - _cyl_y(PILOT / 2, CHEEK_IN - 1, y_floor + 0.1, x=-d)
    # -Y: MR128ZZ pocket from the outer face, 2 mm lip with a clearance hole
    pocket = _cyl_y(TB_POCKET_D / 2, IDLER_FACE_Y - 1, IDLER_FACE_Y + TB_W)
    for a_deg in TB_POCKET_RIB_A:  # crush ribs on the 12.0 OD, like the pan seat
        a = math.radians(a_deg)
        rc = TB_OD / 2 - TB_POCKET_CRUSH + 0.8
        pocket = pocket - _cyl_y(
            0.8,
            IDLER_FACE_Y - 2,
            IDLER_FACE_Y + TB_W + 1,
            x=rc * math.cos(a),
            z=rc * math.sin(a),
        )
    c = c - pocket
    c = c - _cyl_y(4.9, IDLER_FACE_Y, -CHEEK_IN + 1)
    # ribbon tie block above the plate, slots either side of the ribbon, and
    # the strap's groove in the front face (the board sits on that face)
    ty0 = RIBBON_TIE_Y[0] - TIE_SLOT_W / 2
    ty1 = RIBBON_TIE_Y[1] + TIE_SLOT_W / 2
    c = c + _box(
        x_back - PLATE_T, x_back, ty0 - 1.5, ty1 + 1.5, z1 - 0.01, TIE_BLOCK_TOP
    )
    for ty in RIBBON_TIE_Y:
        c = c - _box(
            x_back - PLATE_T - 1,
            x_back + 1,
            ty - TIE_SLOT_W / 2,
            ty + TIE_SLOT_W / 2,
            TIE_SLOT_Z0,
            TIE_SLOT_Z1,
        )
    c = c - _box(x_back - TIE_GROOVE, x_back + 1, ty0, ty1, TIE_SLOT_Z0, TIE_SLOT_Z1)
    # driver channels: from the button heads forward, clear through the cradle
    for h in HOLES:
        c = c - Pos(BOARD_FRONT_X + 1.3, h.Y, h.Z) * (
            Rot(0, 90, 0) * Cylinder(HEAD_DRIVER_D / 2, 40.0, align=(CEN, CEN, MIN))
        )
    return c


HORN_SCREW_D = (10.0, 16.0)  # single-arm horn holes used, from the axis (estimate)


def cable_features_world() -> Part:
    """Ribbon shelf and tilt-lead gutter on the yoke, world frame at pan 0."""
    zl0 = Z_LAND_TOP
    shelf_top = zl0 + SHELF_T
    # shelf: above the land it joins the flange; below it stays clear of the
    # tower's cylinder (r 18.8) by 0.5
    shelf = _box(-SHELF_R1, -FLANGE_R + 1.0, -SHELF_HW, SHELF_HW, zl0, shelf_top)
    shelf += _box(-SHELF_R1, -SHELF_R0, -SHELF_HW, SHELF_HW, zl0 - 1.5, zl0)
    for sy in (-1, 1):  # cable-tie slots either side of the 14 mm ribbon
        shelf -= _box(
            -SHELF_R1 + 1.5,
            -SHELF_R1 + 4.5,
            sy * 8.25 - 0.75,
            sy * 8.25 + 0.75,
            zl0 - 2,
            shelf_top + 1,
        )
    # The gutter is straight, so its walls are planar: its inner face stays
    # 19.3 from the pan axis along its whole length (tower cylinder r 18.8).
    # Local frame: u along the gutter, v outward, centred on the mid angle.
    ri, rc, ro, rx = GUTTER_R
    zg0 = zl0 - 5.0
    am = (GUTTER_A0 + GUTTER_A1) / 2
    hu = ri * math.tan(math.radians((GUTTER_A1 - GUTTER_A0) / 2))
    g = _box(-hu, hu, ri, rx, zg0, zg0 + 1.5)  # floor
    g += _box(-hu, hu, ri, rc, zg0, zl0 + 2.0)  # inner wall
    g += _box(-hu, hu, FLANGE_R - 1.0, rc, zl0, zl0 + 2.0)  # ties it to the flange
    g += _box(-hu, hu, ro, rx, zg0, zg0 + 4.5)  # outer wall
    # one tie slot through the floor near the start (lead + outer wall)
    g -= _box(-hu + 2.0, -hu + 5.0, rc + 0.3, rc + 1.8, zg0 - 1, zg0 + 3)
    g = Rot(0, 0, am - 90.0) * g
    return shelf + g


def build_yoke() -> Part:
    zl = W
    px = -PAN_AXIS_BEHIND_LENS
    # spigot through the 6705ZZ bore, 0.2 short of the race so the keeper clamps
    spigot = _cyl_z(PB_ID / 2 - 0.05, zl(Z_SEAT0 + SPIGOT_SHORT), zl(Z_TOWER_TOP), x=px)
    land = _cyl_z(PB_INNER_RACE_OD / 2 - 0.3, zl(Z_TOWER_TOP), zl(Z_LAND_TOP), x=px)
    zf0, zf1 = zl(Z_LAND_TOP), zl(Z_FLANGE_TOP)
    flange = _cyl_z(FLANGE_R, zf0, zf1, x=px)
    arm_p = _box(-9, 9, 0, UPRIGHT_Y1 + UPRIGHT_RIB_D, zf0, zf1)
    arm_n = _box(-9, 9, FOOT_Y0, 0, zf0, zf1)
    upright = _box(-9, 9, UPRIGHT_Y0, UPRIGHT_Y1, zf0, UPRIGHT_TOP)
    # Ribs beside the tilt servo's case, outboard of the upright: the servo
    # hangs on the upright's outer face, which loads the 3 mm plate in its weak
    # direction. Each rib is 2.4 wide, 0.6 clear of the case and its tabs.
    for sx in (-1, 1):
        x_in = SERVO.width / 2 + 0.6
        upright = upright + _box(
            min(sx * x_in, sx * 9.0),
            max(sx * x_in, sx * 9.0),
            UPRIGHT_Y1 - 0.01,
            UPRIGHT_Y1 + UPRIGHT_RIB_D,
            zf0,
            UPRIGHT_RIB_TOP,
        )
    # gusset between flange arm and upright
    gusset = _box(
        -9,
        9,
        max(UPRIGHT_Y0 - 4.0, BOARD_Y1 + 3.0 + SWEEP_MARGIN + 0.5),
        UPRIGHT_Y0,
        zf0,
        zf1 + 6.0,
    )
    cable = Pos(px, 0, -lens_height()) * cable_features_world()
    y = spigot + land + flange + arm_p + arm_n + upright + gusset + cable
    # tilt servo window (case + FIT), its tab pilots
    x_lo = -SERVO.width / 2 - FIT
    z_lo = SERVO.shaft_from_end - SERVO.length - FIT_END
    y = y - _box(
        x_lo,
        -x_lo,
        UPRIGHT_Y0 - 1,
        UPRIGHT_Y1 + 1,
        z_lo,
        SERVO.shaft_from_end + FIT_END,
    )
    for xs, _, _ in fx.servo_tab_holes(SERVO):
        y = y - _cyl_y(PILOT / 2, UPRIGHT_Y0 - 5, UPRIGHT_Y1 + 1, x=0, z=xs)
    # brow pod mount: 2 x M2 tapped through the upright's top
    for bx in BROW_MOUNT_X:
        y = y - _cyl_y(PILOT / 2, UPRIGHT_Y0 - 1, UPRIGHT_Y1 + 1, x=bx, z=BROW_MOUNT_Z)
    # driver access to the bearing cap's screws: arc slots +-CAP_SLOT_DEG, so
    # they line up whatever spline step the pan horn landed on
    for a in CAP_SCREW_A:
        for da in np.arange(-CAP_SLOT_DEG, CAP_SLOT_DEG + 0.01, 2.0):
            ar = math.radians(a + da)
            y = y - _cyl_z(
                CAP_DRIVER_D / 2,
                zf0 - 1,
                zf1 + 8,
                x=px + CAP_SCREW_R * math.cos(ar),
                y=CAP_SCREW_R * math.sin(ar),
            )
    # tie slot for the right brow's lead (-Y, outrigger side), at the -Y arm's rear corner
    y = y - _box(-8.0, -5.0, -17.0, -15.5, zf0 - 1, zf1 + 1)
    # pan: centre-screw access on the axis; tapped holes for the keeper screws
    y = y - _cyl_z(3.0, zl(Z_SEAT0) - 1, zf1 + 7, x=px)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        y = y - _cyl_z(
            PILOT / 2,
            zl(Z_SEAT0) - 1,
            zl(Z_SEAT0) + 6.0,
            x=px + KEEPER_SCREW_R * math.cos(a),
            y=KEEPER_SCREW_R * math.sin(a),
        )
    # outrigger foot pilots
    for x in OUTRIGGER_SCREW_X:
        y = y - _cyl_z(PILOT / 2, zf0 - 1, zf1 + 1, x=x, y=OUTRIGGER_FOOT_Y)
    # the head and cradle over the whole tilt range
    for tool in sweep_tools():
        y = y - tool
    sols = sorted(y.solids(), key=lambda so: -so.volume)
    for so in sols[1:]:
        print(f"yoke: dropped a {so.volume:.3f} mm3 fragment at {so.center()}")
    return Part([sols[0]])


OUTRIGGER_SCREW_X = (-6.0, 6.0)
UPRIGHT_TOP = 20.0  # was 11.5: carries the left brow pod (+Y)
UPRIGHT_RIB_D, UPRIGHT_RIB_TOP = 6.0, 0.0
# Both pod screws sit forward of the SG90's lower tab (lens x -6.1..-3.6) and
# its tab plate, so their heads clear both and a driver reaches them along Y.
BROW_MOUNT_X, BROW_MOUNT_Z = (0.0, 6.0), 16.0  # lens frame


def _outrigger_foot_y() -> float:
    return OUTRIGGER_PLATE_Y0 - 3.0


OUTRIGGER_FOOT_Y = _outrigger_foot_y()
FOOT_Y0 = OUTRIGGER_FOOT_Y - 3.3 - OUTRIGGER_SLOT + 0.2  # outer edge of foot and -Y arm


def build_outrigger(pin_d: float = TB_PIN_D) -> Part:
    """As printed by default (pin Ø8.25); ``pin_d=TB_ID`` gives it as fitted,
    after the pin is sanded to the bore, which is what assembly() checks."""
    zf1 = W(Z_FLANGE_TOP)
    plate = _box(-7, 7, OUTRIGGER_PLATE_Y0, OUTRIGGER_PLATE_Y1, zf1, 0) + _cyl_y(
        7.0, OUTRIGGER_PLATE_Y0, OUTRIGGER_PLATE_Y1
    )
    # the plate reaches the right brow pod's mount height (x to +9 for the
    # pod's forward screw)
    plate = plate + _box(-7, 9, OUTRIGGER_PLATE_Y0, OUTRIGGER_PLATE_Y1, 0, UPRIGHT_TOP)
    # A shoulder on the pin's root bears on the MR128ZZ's inner race only. It
    # is printed TB_SHOULDER_GAP off the race and set on assembly with a
    # feeler (foot holes slotted +-OUTRIGGER_SLOT in Y), so the cradle (and the
    # tilt horn with it) can walk at most the set gap off the tilt spline. The
    # boss stays TB_SHOULDER_H behind the shoulder, so even fully slid in it
    # stays off the outer race.
    z_sh = IDLER_FACE_Y - TB_SHOULDER_GAP  # shoulder face, as printed
    boss = _cyl_y(6.0, OUTRIGGER_PLATE_Y1 - 0.01, z_sh - TB_SHOULDER_H)
    shoulder = _cyl_y(TB_SHOULDER_R, z_sh - TB_SHOULDER_H - 0.01, z_sh)
    pin = _cyl_y(pin_d / 2, z_sh - 0.01, IDLER_FACE_Y + TB_W - 0.1)
    foot = _box(-9.5, 9.5, FOOT_Y0, OUTRIGGER_PLATE_Y1, zf1, zf1 + 3.0)
    # rib on the outboard face, foot to plate, at x 0 (clear of both screws)
    from build123d import Polyline, extrude, make_face

    rib_pts = [
        (FOOT_Y0, zf1 + 2.9),
        (OUTRIGGER_PLATE_Y0 + 0.01, zf1 + 2.9),
        (OUTRIGGER_PLATE_Y0 + 0.01, zf1 + 18.0),
    ]
    rib = extrude(
        make_face(Polyline(*[(-1.25, y, z) for y, z in rib_pts], close=True)),
        amount=2.5,
        dir=(1, 0, 0),
    )
    o = plate + boss + shoulder + pin + foot + rib
    for x in OUTRIGGER_SCREW_X:  # Ø2.2 clearance, slotted +-OUTRIGGER_SLOT in Y
        for dy in (-OUTRIGGER_SLOT, OUTRIGGER_SLOT):
            o = o - _cyl_z(1.1, zf1 - 1, zf1 + 4, x=x, y=OUTRIGGER_FOOT_Y + dy)
        o = o - _box(
            x - 1.1,
            x + 1.1,
            OUTRIGGER_FOOT_Y - OUTRIGGER_SLOT,
            OUTRIGGER_FOOT_Y + OUTRIGGER_SLOT,
            zf1 - 1,
            zf1 + 4,
        )
    for bx in BROW_MOUNT_X:  # brow pod mount, tapped M2
        o = o - _cyl_y(
            PILOT / 2,
            OUTRIGGER_PLATE_Y0 - 1,
            OUTRIGGER_PLATE_Y1 + 1,
            x=bx,
            z=BROW_MOUNT_Z,
        )
    return o


def _cross(r: float, w: float, z0: float, z1: float) -> Part:
    """Two crossed bars, half-length r and width w, along X and Y."""
    return _box(-r, r, -w / 2, w / 2, z0, z1) + _box(-w / 2, w / 2, -r, r, z0, z1)


ARM_W = 5.0  # fixture cross-horn arm width


def build_keeper(slot_deg: float = 0.0) -> Part:
    """Printed ring under the yoke's spigot (world, about the pan axis): clamps
    the 6705ZZ inner race against the yoke's land, and keys the trimmed cross
    horn's arm stubs in a through cross slot that lets them float axially.

    ``slot_deg`` turns the slot (and its ribs) against the four screw holes.
    The vernier keeper uses 9: fitted either way up it gives +9 or -9 (= +9
    modulo the spline's 18-degree step), so between the two keepers pan zero
    lands within +-4.5 degrees instead of +-9."""
    k = _cyl_z(KEEPER_R, Z_KEEPER0, Z_SEAT0)
    slot = _cross(
        HORN_TRIM_R + KEY_FIT + 0.15, ARM_W + 2 * KEY_FIT, Z_KEEPER0 - 1, Z_SEAT0 + 1
    )
    k = k - (Rot(0, 0, slot_deg) * slot)
    # Crush ribs on both walls of each arm slot, vertical so the arm still
    # slides axially: their crests stand KEY_CRUSH into the arm, which takes
    # out the slot's backlash (the inspector's +-1 degree) for good.
    rib_c = ARM_W / 2 - KEY_CRUSH + KEY_RIB_R
    for kk in range(4):
        for side in (-1, 1):
            rib = _cyl_z(KEY_RIB_R, Z_KEEPER0, Z_SEAT0, x=KEY_RIB_AT, y=side * rib_c)
            k = k + (Rot(0, 0, slot_deg + 90 * kk) * rib)
    k = k - _cyl_z(3.5 + 0.5, Z_KEEPER0 - 1, Z_SEAT0 + 1)  # horn hub
    for kk in range(4):
        a = math.radians(45 + 90 * kk)
        k = k - _cyl_z(
            1.2,
            Z_KEEPER0 - 1,
            Z_SEAT0 + 1,
            x=KEEPER_SCREW_R * math.cos(a),
            y=KEEPER_SCREW_R * math.sin(a),
        )
    return k


def build_cap() -> Part:
    """Bearing cap (world, stationary): clamps the 6705ZZ outer race onto the
    tower's seat floor. Three M2 x 6 button heads in counterbores."""
    c = _cyl_z(CAP_R, Z_TOWER_TOP, Z_CAP_TOP) - _cyl_z(
        CAP_BORE_R, Z_TOWER_TOP - 1, Z_CAP_TOP + 1
    )
    for a in CAP_SCREW_A:
        x, y = (
            CAP_SCREW_R * math.cos(math.radians(a)),
            CAP_SCREW_R * math.sin(math.radians(a)),
        )
        c = c - _cyl_z(1.1, Z_TOWER_TOP - 1, Z_CAP_TOP + 1, x=x, y=y)
        c = c - _cyl_z(CAP_CB_D / 2, Z_CAP_TOP - CAP_CB_DEPTH, Z_CAP_TOP + 1, x=x, y=y)
    return c


def cap_screws_world() -> list[Part]:
    out = []
    for a in CAP_SCREW_A:
        x, y = (
            CAP_SCREW_R * math.cos(math.radians(a)),
            CAP_SCREW_R * math.sin(math.radians(a)),
        )
        z_head = Z_CAP_TOP - CAP_CB_DEPTH
        out.append(_cyl_z(SCREW_CORE / 2, z_head - 6.0, z_head, x=x, y=y))
        out.append(_cyl_z(1.75, z_head, z_head + 1.3, x=x, y=y))
    return out


def build_tower() -> Part:
    """Stationary: chassis flange, servo box (tabs screw up into the ledge),
    horn chamber, 6705ZZ seat with crush ribs, ribbon post. World coordinates
    about the pan axis."""
    s = SERVO
    cx = s.shaft_from_end - s.length / 2  # case centre, servo x (= world x)
    half_tab = s.tab_span / 2
    tab_x0, tab_x1 = cx - half_tab - FIT_TAB, cx + half_tab + FIT_TAB
    box_x0, box_x1 = tab_x0 - WALL, tab_x1 + WALL
    hw = s.width / 2 + FIT
    tab_top = ZS - (s.height - s.tab_underside) + s.tab_thickness
    body = _box(box_x0, box_x1, -hw - WALL, hw + WALL, 0, CASE_TOP_CLEAR + 2.0)
    cyl = _cyl_z(TOWER_R, tab_top, Z_TOWER_TOP)
    flange = _box(-25.0, 19.0, -20.0, 20.0, 0, CHASSIS_FLANGE_T)
    # ribbon post at the rear, on a tongue of the flange
    tongue = _box(POST_X0, -24.0, -POST_HW, POST_HW, 0, CHASSIS_FLANGE_T)
    post = _box(POST_X0, POST_X1, -POST_HW, POST_HW, 0, POST_TOP)
    bosses = [_cyl_z(M3_BOSS_R, 0, M3_BOSS_TOP, x=x, y=y) for x, y in M3_HOLES]
    t = body + cyl + flange + tongue + post
    for b in bosses:
        t = t + b
    # tab-span cavity open at the bottom, case cavity above the ledge
    t = t - _box(tab_x0, tab_x1, -hw, hw, -1, tab_top)
    t = t - _box(
        s.shaft_from_end - s.length - FIT_END,
        s.shaft_from_end + FIT_END,
        -hw,
        hw,
        tab_top - 1,
        CASE_TOP_CLEAR,
    )
    t = t - _cyl_z(CHAMBER_R, Z_CHAMBER, Z_SEAT0 + 0.01)
    seat = _cyl_z(SEAT_R, Z_SEAT0, Z_TOWER_TOP + 1)
    for a_deg in RIB_A:  # crush ribs, 0.15 interference on the 32.0 OD
        a = math.radians(a_deg)
        rc = PB_OD / 2 - CRUSH + 0.8
        seat = seat - _cyl_z(
            0.8, Z_SEAT0 - 1, Z_TOWER_TOP + 2, x=rc * math.cos(a), y=rc * math.sin(a)
        )
    t = t - seat
    # lead exit slot at the -X end, open to the bottom
    boot_x0 = s.shaft_from_end - s.length - 6.0 - 1.0
    t = t - _box(boot_x0 - 5, tab_x0 + 0.5, -3.0, 3.0, -1, ZS - s.height + 10.0)
    # tab holes, tapped M2 from below
    for x, _, z in fx.servo_tab_holes(s):
        t = t - _cyl_z(
            PILOT / 2, tab_top - 0.01, min(tab_top + 6.5, Z_CHAMBER + 0.5), x=x
        )
    for x, y in M3_HOLES:  # M3 clearance + captive nut pocket from the top
        t = t - _cyl_z(1.7, -1, M3_BOSS_TOP + 1, x=x, y=y)
        t = t - Pos(x, y, M3_BOSS_TOP - M3_NUT_DEPTH) * _hex_prism(
            M3_NUT_AF, M3_NUT_DEPTH + 1
        )
    for a in CAP_SCREW_A:  # bearing cap screws, tapped M2 x 0.4
        t = t - _cyl_z(
            PILOT / 2,
            Z_TOWER_TOP - 6.5,
            Z_TOWER_TOP + 1,
            x=CAP_SCREW_R * math.cos(math.radians(a)),
            y=CAP_SCREW_R * math.sin(math.radians(a)),
        )
    # cable-tie slots through the post (along X), either side of the ribbon
    for z in (12.0, POST_TOP - 8.0):
        for sy in (-1, 1):
            t = t - _box(
                POST_X0 - 1,
                POST_X1 + 1,
                sy * 8.25 - 0.75,
                sy * 8.25 + 0.75,
                z - 1.5,
                z + 1.5,
            )
    return t


# --------------------------------------------------------------------------
# Stand-ins
# --------------------------------------------------------------------------
def tilt_servo_bodies() -> list[Part]:
    sv = TILT_SERVO_LOC * fx.servo(SERVO)
    screws = []
    for xs, ys, zs in fx.servo_tab_holes(SERVO):
        # shank through tab + upright (+2.5 tip), head on the tab
        screws.append(
            _cyl_y(
                SCREW_CORE / 2, UPRIGHT_Y0 - 2.5, TAB_IN_Y + SERVO.tab_thickness, z=xs
            )
        )
        screws.append(
            _cyl_y(
                1.9,
                TAB_IN_Y + SERVO.tab_thickness,
                TAB_IN_Y + SERVO.tab_thickness + 1.5,
                z=xs,
            )
        )
    return [sv] + screws


def tilt_horn_bodies() -> list[Part]:
    h = TILT_SERVO_LOC * (Rot(0, 0, 90) * bored_horn("single"))
    y_face = CHEEK_IN + HORN_CHEEK_T
    y_floor = y_face - HORN_POCKET
    screws = []
    for d in HORN_SCREW_D:
        screws.append(_cyl_y(SCREW_CORE / 2, CHEEK_IN + 0.6, y_floor + 1.6, x=-d))
        screws.append(_cyl_y(1.6, y_floor + 1.6, y_floor + 1.6 + 1.2, x=-d))
    return [h] + screws


def tilt_bearing() -> Part:
    return _cyl_y(TB_OD / 2, IDLER_FACE_Y, IDLER_FACE_Y + TB_W) - _cyl_y(
        TB_ID / 2, IDLER_FACE_Y - 1, IDLER_FACE_Y + TB_W + 1
    )


def head_bodies() -> list[Part]:
    parts = [HEAD_LOC * fx.head(screws=True), _box(*RIBBON_ON_HEAD)]
    for h in HOLES:  # M2 x 5 button head: shank from the board into the insert
        parts.append(
            Pos(h.X - PLATE_T + 0.4, h.Y, h.Z)
            * (Rot(0, 90, 0) * Cylinder(0.85, PLATE_T - 0.4, align=(CEN, CEN, MIN)))
        )
    return parts


def pan_servo_world() -> list[Part]:
    sv = Pos(0, 0, ZS) * fx.servo(SERVO)
    tab_top = ZS - (SERVO.height - SERVO.tab_underside) + SERVO.tab_thickness
    screws = []
    for x, _, z in fx.servo_tab_holes(SERVO):
        screws.append(_cyl_z(SCREW_CORE / 2, tab_top, tab_top + 5.0, x=x))
        screws.append(_cyl_z(1.9, ZS + z - 1.5, ZS + z, x=x))
    return [sv] + screws


def pan_horn_world() -> list[Part]:
    """Stock cross horn, arms cut to HORN_TRIM_R, bored on the spline, and its
    centre screw (head on the hub)."""
    h = Pos(0, 0, ZS) * bored_horn("cross")
    h = h & _cyl_z(HORN_TRIM_R, ZS - 1, ZS + 10)
    centre = _cyl_z(2.2, Z_HORN_TOP, Z_HORN_TOP + 1.4)  # pan head, ~Ø4.4 x 1.4
    return [h, centre]


def keeper_screws_world() -> list[Part]:
    """4 x M2 x 8 button head up through the keeper into the spigot."""
    out = []
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = KEEPER_SCREW_R * math.cos(a), KEEPER_SCREW_R * math.sin(a)
        out.append(_cyl_z(SCREW_CORE / 2, Z_KEEPER0, Z_KEEPER0 + 8.0, x=x, y=y))
        out.append(_cyl_z(1.75, Z_KEEPER0 - 1.3, Z_KEEPER0, x=x, y=y))
    return out


def chassis_plate_world() -> Part:
    """The acrylic 2WD plate the tower stands on (a stand-in, not printed)."""
    return _box(-45.0, 45.0, -45.0, 45.0, -CHASSIS_PLATE_T, 0.0)


def ribbon_clamped_world() -> tuple[Part, Part]:
    """The two tied-down stretches of the IDC ribbon: on the yoke's shelf
    (pans) and on the tower's post (stationary). Pan 0."""
    shelf_top = Z_LAND_TOP + SHELF_T
    hw = RIBBON_W / 2
    on_shelf = _box(
        -SHELF_R1, -SHELF_R1 + 6.0, -hw, hw, shelf_top, shelf_top + RIBBON_T
    )
    on_post = _box(POST_X0 - RIBBON_T, POST_X0, -hw, hw, 8.0, POST_TOP)
    return on_shelf, on_post


def pan_bearing_world() -> Part:
    return _cyl_z(PB_OD / 2, Z_SEAT0, Z_TOWER_TOP) - _cyl_z(
        PB_ID / 2, Z_SEAT0 - 1, Z_TOWER_TOP + 1
    )


def _hex_prism(af: float, h: float) -> Part:
    from build123d import RegularPolygon, extrude

    return extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=h)


def chassis_screws_world() -> list[Part]:
    """M3 x 10 socket heads from under the acrylic, nuts captive in the bosses."""
    out = []
    z_under = -CHASSIS_PLATE_T
    for x, y in M3_HOLES:
        out.append(_cyl_z(2.75, z_under - 3.0, z_under, x=x, y=y))  # head
        out.append(_cyl_z(1.45, z_under, z_under + 10.0, x=x, y=y))
        out.append(Pos(x, y, M3_BOSS_TOP - 2.4) * _hex_prism(5.5, 2.4))  # nut
    return out


# --------------------------------------------------------------------------
# Optional eyebrows (graft from teuvo-face: brows on the fixed pan-stage
# forehead). Each SG90 sits in a printed pod bolted to the yoke's upright (+Y)
# or the outrigger (-Y), so the brows pan with the head but do not tilt: the
# tilt servo carries only the camera, no brow lead crosses the tilt joint, and
# the eye rolls under the brows. Lens frame throughout.
#
# SG90 frame -> lens frame: shaft (servo z) forward along +X, case long axis
# (servo x) down along -Z, so the shaft is at the case's BOTTOM end (see
# _brow_loc), servo y along +Y.
# --------------------------------------------------------------------------
BROW_SERVO = fx.SG90
BROW_X_TOP = 7.0  # SG90 case top (shaft end), lens frame x: the brow clears the upright
BROW_PIVOT_Z = 26.0  # shaft axis height above the lens
BROW_REST_DEG = 12.0  # inner end this far below horizontal at rest
BROW_TIP_FROM_LENS = 4.0  # inner tip, |y| from the lens centre line
BROW_POD_T = 3.0  # pod mount plate and tab plate
BROW_FIT = 0.3
BROW_TRAVEL = (
    -40.0,
    40.0,
)  # brow angles checked clear at every tilt (scratch sweep, README)


def _brow_frame(side: int) -> tuple[float, float]:
    """(mount face |y|, shaft |y|): side +1 is +Y (on the upright), -1 is -Y
    (on the outrigger)."""
    face = UPRIGHT_Y1 if side > 0 else -OUTRIGGER_PLATE_Y0
    return face, face + BROW_POD_T + 0.5 + BROW_SERVO.width / 2


def _brow_loc(side: int) -> Location:
    """SG90 frame -> lens frame: shaft (servo z) forward along +X, case long
    axis (servo x) DOWN along -Z, so the shaft is at the case's lower end and
    the case and tabs stand above it, clear of the tilt servo; servo y -> +Y."""
    _, yb = _brow_frame(side)
    return Location(
        Plane(
            origin=(BROW_X_TOP, side * yb, BROW_PIVOT_Z),
            x_dir=(0, 0, -1),
            z_dir=(1, 0, 0),
        )
    )


def _brow_arm_deg(side: int, brow_deg: float = 0.0) -> float:
    """Horn rotation about the SG90 shaft (servo frame) so the arm points
    inward and BROW_REST_DEG - brow_deg below horizontal (positive brow_deg
    raises the inner end). Servo +x is world down, servo +y world +Y."""
    down = BROW_REST_DEG - brow_deg
    return -90.0 + down if side > 0 else 90.0 - down


def _brow_case_z() -> tuple[float, float, float]:
    """(case bottom, case top, tab-span centre) in lens z."""
    sv = BROW_SERVO
    zc = BROW_PIVOT_Z - (sv.shaft_from_end - sv.length / 2)
    return zc - sv.length / 2, zc + sv.length / 2, zc


def build_brow_pod(side: int) -> Part:
    """Pod: a mount plate against the upright / outrigger (2 x M2 into their
    tapped holes) and, square to it, a tab plate the SG90's tabs screw to
    (2 x M2 tapped), with a window the case passes through. Built on +Y and
    mirrored for -Y."""
    sv = BROW_SERVO
    face, yb = _brow_frame(side)
    x_tab0 = BROW_X_TOP - (sv.height - sv.tab_underside)  # tab underside, lens x
    z0c, z1c, zc = _brow_case_z()
    half_tab = sv.tab_span / 2
    tz0, tz1 = max(zc - half_tab - 1.5, 11.5), zc + half_tab + 1.5
    tab_plate = _box(
        x_tab0 - BROW_POD_T, x_tab0, face, yb + sv.width / 2 + 3.0, tz0, tz1
    )
    # mount plate: above the tilt servo's top tab (z 10.45) and its screw head
    mount = _box(x_tab0 - BROW_POD_T, 9.0, face, face + BROW_POD_T, 11.5, tz1)
    pod = tab_plate + mount
    pod -= _box(
        x_tab0 - BROW_POD_T - 1,
        x_tab0 + 1,
        yb - sv.width / 2 - BROW_FIT,
        yb + sv.width / 2 + BROW_FIT,
        z0c - BROW_FIT,
        z1c + BROW_FIT,
    )
    for xs, _, _ in fx.servo_tab_holes(sv):  # tab screws, tapped M2
        pod -= _cyl_x(
            PILOT / 2, x_tab0 - BROW_POD_T - 1, x_tab0 + 1, y=yb, z=BROW_PIVOT_Z - xs
        )
    for bx in BROW_MOUNT_X:  # clearance for the M2 x 6 into the upright
        pod -= _cyl_y(1.1, face - 1, face + BROW_POD_T + 1, x=bx, z=BROW_MOUNT_Z)
    # lead slot: the SG90's lead leaves the case top; it drops down the pod's
    # outboard edge, tied through this 1.5 x 3 slot (a 2.5 mm tie passes)
    pod -= _box(
        x_tab0 - BROW_POD_T - 1,
        x_tab0 + 1,
        yb + sv.width / 2 + 0.3,
        yb + sv.width / 2 + 1.8,
        tz0 + 3.0,
        tz0 + 6.0,
    )
    if side < 0:
        pod = pod.mirror(Plane.XZ)
    return pod


def _cyl_x(r, x0, x1, y=0.0, z=0.0) -> Part:
    return Pos(x0, y, z) * (Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=(CEN, CEN, MIN)))


BROW_LIP = 1.2


def build_brow(side: int) -> Part:
    """The brow, in the SG90 horn's frame (servo frame, arm along +X): a 2.5 mm
    plate in front of the horn arm, side lips that key it on the arm, two
    pilots for the horn's own screws (driven from behind through the arm), a
    hole over the centre screw. The inner 13 mm is the brow proper, 10 mm tall."""
    _, yb = _brow_frame(side)
    length = (yb - BROW_TIP_FROM_LENS) / math.cos(math.radians(BROW_REST_DEG))
    za, zb = 4.6, 7.1  # plate: 0.1 in front of the arm (arm 2.9..4.5)
    w = 5.5 / 2 + 0.3  # arm half width + clearance
    plate = _box(-5.0, length, -(w + BROW_LIP), w + BROW_LIP, za, zb)
    brow = _box(length - 13.0, length, -5.0, 5.0, za, zb + 0.5)
    lips = _box(4.5, 20.0, w, w + BROW_LIP, 2.9, za + 0.01) + _box(
        4.5, 20.0, -(w + BROW_LIP), -w, 2.9, za + 0.01
    )
    b = plate + brow + lips
    b -= Pos(0, 0, 0) * Cylinder(3.75 + 0.25, 2 * za, align=(CEN, CEN, CEN))  # hub
    b -= Cylinder(2.6, 20, align=(CEN, CEN, CEN))  # centre screw access
    for d in (8.0, 14.0):
        b -= Pos(d, 0, 0) * Cylinder(PILOT / 2, 20, align=(CEN, CEN, CEN))
    return b


def brow_bodies(side: int, brow_deg: float = 0.0) -> dict:
    """Lens-frame bodies of one brow unit at a brow angle: pod (printed),
    SG90 + screws, horn, brow (printed)."""
    loc = _brow_loc(side)
    sv = BROW_SERVO
    arm = Rot(0, 0, _brow_arm_deg(side, brow_deg))
    spline = Cylinder(sv.spline_d / 2, sv.overall - sv.height, align=(CEN, CEN, MIN))
    horn = loc * (arm * (fx.horn("single", sv) - spline))
    face, yb = _brow_frame(side)
    x_tab0 = BROW_X_TOP - (sv.height - sv.tab_underside)
    screws = []
    for xs, _, _ in fx.servo_tab_holes(sv):  # M2 x 6 through the tab into the plate
        z = BROW_PIVOT_Z - xs
        x_head = x_tab0 + sv.tab_thickness
        screws.append(
            _cyl_x(SCREW_CORE / 2, x_tab0 - BROW_POD_T - 0.5, x_head, y=side * yb, z=z)
        )
        screws.append(_cyl_x(1.9, x_head, x_head + 1.5, y=side * yb, z=z))
    mount = []
    for bx in BROW_MOUNT_X:  # M2 x 6 through the pod into the upright / outrigger
        y_out = side * (face + BROW_POD_T)
        y_in = side * (face - 2.9)
        mount.append(
            _cyl_y(
                SCREW_CORE / 2, min(y_in, y_out), max(y_in, y_out), x=bx, z=BROW_MOUNT_Z
            )
        )
        y_h = side * (face + BROW_POD_T + 1.3)
        mount.append(
            _cyl_y(1.75, min(y_out, y_h), max(y_out, y_h), x=bx, z=BROW_MOUNT_Z)
        )
    return {
        "pod": build_brow_pod(side),
        "servo": loc * fx.servo(sv),
        "screws": screws,
        "mount_screws": mount,
        "horn": horn,
        "brow": loc * (arm * build_brow(side)),
    }


# --------------------------------------------------------------------------
# The ribbon, taut, per pose
# --------------------------------------------------------------------------
# The spare ribbon in a free span has no single shape, but one shape is
# determinate and is the one that snags: the span pulled taut, which is what
# the longest-chord poses come close to (README: leave 55 and 55 mm). Both
# free spans are modelled as their taut chord at every pose and swept by the
# clash gate with everything else. The tilt span bends about its width, so it
# is a 14 x 1.2 strip; the pan span twists, so it is a 3 mm spine.
RIBBON_END_TRIM = 4.0  # mm left out at each end, where the span meets its anchor
RIBBON_SPINE_R = 0.75  # the pan span's centre line, as thick as a ribbon edge-on


def _trimmed(a: np.ndarray, b: np.ndarray, t0: float, t1: float):
    u = (b - a) / np.linalg.norm(b - a)
    return a + u * t0, b - u * t1


def strip_between(a: np.ndarray, b: np.ndarray, width: float, thick: float) -> Part:
    """A flat strip from a to b, its width along Y (a and b in an XZ plane)."""
    d = b - a
    ln = float(np.linalg.norm(d))
    ang = math.degrees(math.atan2(-d[2], d[0]))
    yaw = math.degrees(math.atan2(d[1], math.hypot(d[0], d[2])))
    mid = (a + b) / 2
    return Pos(*mid) * (Rot(0, ang, 0) * (Rot(0, 0, yaw) * Box(ln, width, thick)))


def ribbon_tilt_ends(tilt_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Tilt span ends in the lens frame (pan 0): the tie on the cradle's back
    and the tie on the shelf."""
    lh = lens_height()
    q = (Rot(0, -tilt_deg, 0) * Pos(*RIBBON_ANCHOR)).position
    a = np.array([q.X, q.Y, q.Z])
    shelf_top = Z_LAND_TOP + SHELF_T + RIBBON_T / 2
    b = np.array([-SHELF_R1 + 6.0 - PAN_AXIS_BEHIND_LENS, 0.0, shelf_top - lh])
    return a, b


def ribbon_tilt_taut(tilt_deg: float) -> Part:
    a, b = ribbon_tilt_ends(tilt_deg)
    a, b = _trimmed(a, b, RIBBON_END_TRIM, RIBBON_END_TRIM)
    return strip_between(a, b, RIBBON_W, RIBBON_T)


def ribbon_pan_taut(pan_deg: float) -> Part | None:
    """World frame: shelf rear edge (pans) to the post's top tie (fixed)."""
    shelf_top = Z_LAND_TOP + SHELF_T + RIBBON_T / 2
    # leaves the shelf's rear edge just above the ribbon lying on it
    e = (
        Rot(0, 0, pan_deg)
        * Pos(-SHELF_R1, 0, shelf_top + RIBBON_T / 2 + RIBBON_SPINE_R + 0.1)
    ).position
    a = np.array([e.X, e.Y, e.Z])
    b = np.array(
        [POST_X1, 0.0, POST_TOP + RIBBON_SPINE_R + 0.1]
    )  # over the post's top front edge
    if np.linalg.norm(b - a) < 2 * RIBBON_END_TRIM + 1.0:
        return None  # ends nearly meet (pan 0): nothing between the trimmed ends
    a, b = _trimmed(a, b, RIBBON_END_TRIM, RIBBON_END_TRIM)
    d = b - a
    ln = float(np.linalg.norm(d))
    u = d / ln
    rot = Location(Plane(origin=tuple(a), z_dir=tuple(u)))
    return rot * Cylinder(RIBBON_SPINE_R, ln, align=(CEN, CEN, MIN))


# --------------------------------------------------------------------------
# Assembly
# --------------------------------------------------------------------------
def build_coupon(tower: Part, cradle: Part) -> Part:
    """Bench coupon of the two thinnest webs, cut from the real parts:

    - a slice of the tower's servo ledge round the +X MG90D tab hole, with the
      case window's edge beside it (the 1.0 mm web; the yoke's tilt-tab holes
      are the same hole, window and fit), and
    - the cradle plate round its +Y insert hole, with the tie slot and strap
      groove above it (the cradle's thinnest web was insert-to-slot),

    joined by a sprue. Drill, tap and drive an M2 into the first; set an insert
    with the iron in the second. Its pose is its own (flat), not in assembly()."""
    s = SERVO
    hx = max(x for x, _, _ in fx.servo_tab_holes(s))
    hw = s.width / 2 + FIT
    tab_top = ZS - (s.height - s.tab_underside) + s.tab_thickness
    # top at the horn chamber's floor: higher, the chamber's r 14.3 bore would
    # leave a knife edge where it meets the coupon's cut face
    ledge = tower & _box(
        hx - 5.0, hx + 5.0, -hw - WALL - 0.1, hw + WALL + 0.1, tab_top, Z_CHAMBER
    )
    xb = BOARD_BACK_X
    piece = cradle & _box(
        xb - PLATE_T - 0.1, xb + 0.1, 8.0, 18.5, PLATE_Z0 - 0.1, TIE_BLOCK_TOP + 0.1
    )
    gap = 4.0
    anchor = Vector(xb - PLATE_T, 9.05, -10.5)
    target = Vector(hx + 5.0 + gap, 0.0, tab_top + 1.5)
    piece = Pos(target - anchor) * piece
    bar = _box(
        hx + 4.5, hx + 5.0 + gap + 0.5, -0.75, 0.75, tab_top + 0.75, tab_top + 2.25
    )
    c = ledge + bar + piece
    sols = c.solids()
    assert len(sols) == 1, f"coupon is {len(sols)} solids"
    return Part([sols[0]])


def to_world() -> Location:
    return Pos(PAN_AXIS_BEHIND_LENS, 0, lens_height())


@functools.lru_cache(maxsize=1)
def _built() -> dict:
    return {
        "cradle": build_cradle(),
        "yoke": build_yoke(),
        "outrigger": build_outrigger(),
        "outrigger_fitted": build_outrigger(TB_ID),
        "tower": build_tower(),
        "keeper": build_keeper(),
        "keeper_9deg": build_keeper(9.0),
        "bearing_cap": build_cap(),
        "brow_pod_l": build_brow_pod(+1),  # +Y is Teuvo's left
        "brow_pod_r": build_brow_pod(-1),
        "brow_l": build_brow(+1),
        "brow_r": build_brow(-1),
    }


@functools.lru_cache(maxsize=1)
def _coupon() -> Part:
    b = _built()
    return build_coupon(b["tower"], b["cradle"])


def printed_parts() -> dict[str, Part]:
    """Every printed part in its WORLD pose at pan 0, tilt 0, except the bench
    coupon. The outrigger is as printed (pin Ø8.25); assembly() checks it as
    fitted, with the pin sanded to the 8.0 bore."""
    b = _built()
    w = to_world()
    return {
        "tower": b["tower"],
        "yoke": w * b["yoke"],
        "outrigger": w * b["outrigger"],
        "cradle": w * b["cradle"],
        "keeper": b["keeper"],
        "keeper_9deg": b["keeper_9deg"],
        "bearing_cap": b["bearing_cap"],
        # optional eyebrow parts, world pose at pan 0 (brows at rest)
        "brow_pod_l": w * b["brow_pod_l"],
        "brow_pod_r": w * b["brow_pod_r"],
        "brow_l": w * (_brow_loc(+1) * (Rot(0, 0, _brow_arm_deg(+1)) * b["brow_l"])),
        "brow_r": w * (_brow_loc(-1) * (Rot(0, 0, _brow_arm_deg(-1)) * b["brow_r"])),
        # bench coupon, in its own pose (not part of the assembly)
        "coupon": _coupon(),
    }


REQUIRED = ("tower", "yoke", "keeper", "bearing_cap", "outrigger", "cradle")
OPTIONAL = ("keeper_9deg", "brow_pod_l", "brow_pod_r", "brow_l", "brow_r", "coupon")


COLOURS = {
    "tower": "#3d3d3d",
    "yoke": "#b54a68",
    "outrigger": "#d9822b",
    "cradle": "#3a7ca5",
    "keeper": "#7a7a7a",
    "bearing_cap": "#5a5a5a",
    "brow_pod": "#3d3d3d",
    "brow": "#1f1f1f",
}


def assembly(pan_deg: float, tilt_deg: float) -> list[Body]:
    b = _built()
    w = to_world()
    pan = Rot(0, 0, pan_deg)
    tilt = Rot(0, -tilt_deg, 0)  # positive tilt = nose up

    def P(part):
        return pan * w * part

    def T(part):
        return pan * w * tilt * part

    out = [
        Body("tower", b["tower"], "tower", printed=True, colour=COLOURS["tower"]),
        Body("pan_bearing", pan_bearing_world(), "tower"),
        Body("chassis_plate", chassis_plate_world(), "tower"),
    ]
    rib_shelf, rib_post = ribbon_clamped_world()
    out += [Body("ribbon_post", rib_post, "tower")]
    out += [
        Body(f"chassis_screw{i}", s, "tower")
        for i, s in enumerate(chassis_screws_world())
    ]
    out += [
        Body(f"pan_servo{i}" if i else "pan_servo", s, "pan_servo")
        for i, s in enumerate(pan_servo_world())
    ]
    out += [
        Body(f"pan_horn{i}" if i else "pan_horn", pan * s, "pan_horn")
        for i, s in enumerate(pan_horn_world())
    ]
    out += [Body("yoke", P(b["yoke"]), "yoke", printed=True, colour=COLOURS["yoke"])]
    out += [
        Body(
            "keeper", pan * b["keeper"], "yoke", printed=True, colour=COLOURS["keeper"]
        )
    ]
    out += [
        Body(f"keeper_screw{i}", pan * s, "yoke")
        for i, s in enumerate(keeper_screws_world())
    ]
    out += [Body("ribbon_shelf", pan * rib_shelf, "yoke")]
    out += [
        Body(
            "bearing_cap",
            b["bearing_cap"],
            "tower",
            printed=True,
            colour=COLOURS["bearing_cap"],
        )
    ]
    out += [Body(f"cap_screw{i}", s, "tower") for i, s in enumerate(cap_screws_world())]
    # the keeper's slot ribs crush into the horn's arm stubs by design
    for o in out:
        if o.name == "keeper":
            o.allow = {"pan_horn"}
    # Both free spans of the ribbon, taut, each its own group. They are gate
    # proxies, not parts, and matplotlib's per-body depth sort lets a long
    # translucent strip wash the head out of every view, so common.quickview
    # leaves them out of the pictures; common.check always has them.
    if "common.quickview" not in sys.modules:
        out += [Body("ribbon_tilt_taut", P(ribbon_tilt_taut(tilt_deg)), "ribbon_tilt")]
        rp = ribbon_pan_taut(pan_deg)
        if rp is not None:
            out += [Body("ribbon_pan_taut", rp, "ribbon_pan")]
    if BROWS:
        for side, tag in ((+1, "l"), (-1, "r")):  # +Y is Teuvo's left
            bb = brow_bodies(side)
            g = f"brow_{tag}"
            out += [
                Body(
                    f"brow_pod_{tag}",
                    P(bb["pod"]),
                    g,
                    printed=True,
                    colour=COLOURS["brow_pod"],
                ),
                Body(f"brow_servo_{tag}", P(bb["servo"]), g),
                Body(f"brow_horn_{tag}", P(bb["horn"]), g),
                Body(
                    f"brow_{tag}",
                    P(bb["brow"]),
                    g,
                    printed=True,
                    colour=COLOURS["brow"],
                ),
            ]
            out += [
                Body(f"brow_screw_{tag}{i}", P(s), g)
                for i, s in enumerate(bb["screws"])
            ]
            # The pod's mount screws are their own group, so the gate checks
            # them against the SG90, its horn and the pod (inside the brow
            # group nothing is tested). The SG90's tab screws stay in the brow
            # group: the fixture's tabs have no holes for them to pass.
            out += [
                Body(f"brow_mount_screw_{tag}{i}", P(s), f"brow_mount_{tag}")
                for i, s in enumerate(bb["mount_screws"])
            ]
    out += [
        Body(f"tilt_servo{i}" if i else "tilt_servo", P(s), "tilt_servo")
        for i, s in enumerate(tilt_servo_bodies())
    ]
    out += [
        Body(
            "outrigger",
            P(b["outrigger_fitted"]),  # pin sanded to the bore (printed oversize)
            "outrigger",
            printed=True,
            colour=COLOURS["outrigger"],
        )
    ]
    out += [
        Body(
            f"outrigger_screw{i}",
            P(
                _cyl_z(
                    SCREW_CORE / 2,
                    W(Z_FLANGE_TOP) - 3.5,
                    W(Z_FLANGE_TOP) + 3.0,
                    x=x,
                    y=OUTRIGGER_FOOT_Y,
                )
                + _cyl_z(
                    1.9,
                    W(Z_FLANGE_TOP) + 3.0,
                    W(Z_FLANGE_TOP) + 4.3,
                    x=x,
                    y=OUTRIGGER_FOOT_Y,
                )
            ),
            "outrigger",
        )
        for i, x in enumerate(OUTRIGGER_SCREW_X)
    ]
    out += [
        Body("cradle", T(b["cradle"]), "cradle", printed=True, colour=COLOURS["cradle"])
    ]
    out += [Body("tilt_bearing", T(tilt_bearing()), "cradle")]
    out += [
        Body(f"tilt_horn{i}" if i else "tilt_horn", T(s), "tilt_horn")
        for i, s in enumerate(tilt_horn_bodies())
    ]
    hb = head_bodies()
    out += [Body("head", T(hb[0]), "head"), Body("ribbon_on_head", T(hb[1]), "head")]
    out += [Body(f"head_screw{i}", T(s), "head") for i, s in enumerate(hb[2:])]
    for o in out:  # one ribbon either side of its shelf tie touching itself
        if o.name == "ribbon_tilt_taut":
            o.allow = {"ribbon_pan_taut"}
    return out


# --------------------------------------------------------------------------
# Centre of mass of everything that pans
# --------------------------------------------------------------------------
def panned_com() -> tuple[Vector, float]:
    """Panned CoM in the lens frame, and the panned mass (g)."""
    b = _built()
    items: list[tuple[float, Vector]] = []
    for name in ("yoke", "outrigger", "cradle"):
        p = b[name]
        items.append((p.volume * DENSITY_PA12, p.center()))
    k = b["keeper"]
    items.append(
        (
            k.volume * DENSITY_PA12,
            k.center() - Vector(PAN_AXIS_BEHIND_LENS, 0, lens_height()),
        )
    )
    items.append((M_SERVO, (TILT_SERVO_LOC * fx.servo(SERVO, cable=False)).center()))
    items.append((M_HORN, Vector(-6, CHEEK_IN + HORN_CHEEK_T - 1, 0)))
    items.append((M_HORN, Vector(-PAN_AXIS_BEHIND_LENS, 0, W(ZS + 3))))
    items.append((M_TB, Vector(0, IDLER_FACE_Y + TB_W / 2, 0)))
    items.append((M_INSERTS, Vector(BOARD_BACK_X - 1.5, 0, HOLE_Z)))
    items.append((M_HEAD_BOARD, hp(fx.HEAD_THICK / 2, 0, fx.HEAD_SIZE / 2)))
    xs, _ys, _zs = fx.XIAO_STACK
    items.append(
        (
            M_XIAO,
            hp(fx.HEAD_THICK + xs * 0.45, fx.XIAO_CENTRE_YZ[0], fx.XIAO_CENTRE_YZ[1]),
        )
    )
    items.append((M_PLUG, hp(-5.0, fx.J2_CENTRE_YZ[0], fx.J2_CENTRE_YZ[1])))
    m = sum(w for w, _ in items)
    c = sum((v * w for w, v in items), Vector(0, 0, 0)) * (1 / m)
    return c, m


def tilting_com() -> tuple[Vector, float]:
    b = _built()
    items = [
        (b["cradle"].volume * DENSITY_PA12, b["cradle"].center()),
        (M_HORN, Vector(-6, 13, 0)),
        (M_TB, Vector(0, IDLER_FACE_Y + TB_W / 2, 0)),
        (M_INSERTS, Vector(BOARD_BACK_X - 1.5, 0, HOLE_Z)),
        (M_HEAD_BOARD, hp(fx.HEAD_THICK / 2, 0, fx.HEAD_SIZE / 2)),
        (
            M_XIAO,
            hp(
                fx.HEAD_THICK + fx.XIAO_STACK[0] * 0.45,
                fx.XIAO_CENTRE_YZ[0],
                fx.XIAO_CENTRE_YZ[1],
            ),
        ),
        (M_PLUG, hp(-5.0, fx.J2_CENTRE_YZ[0], fx.J2_CENTRE_YZ[1])),
    ]
    m = sum(w for w, _ in items)
    return sum((v * w for w, v in items), Vector(0, 0, 0)) * (1 / m), m


def ribbon_spans() -> dict:
    """Chord lengths of the ribbon's two free spans over the full sweep.

    Tilt span: from the ribbon's tie on the cradle's back (RIBBON_ANCHOR) to
    where it is tied on the yoke's shelf. Pan span: from the shelf's rear
    edge to the post's top front edge, over which it drops to the post ties. A free span must be at least its longest chord, plus a
    bend allowance; the difference to the shortest chord is the slack that
    has to have somewhere to go."""
    lh = lens_height()
    shelf_top = Z_LAND_TOP + SHELF_T + RIBBON_T / 2
    plug = Vector(*RIBBON_ANCHOR)
    tie = Vector(-SHELF_R1 + 6.0 - PAN_AXIS_BEHIND_LENS, 0, shelf_top - lh)
    tilt = []
    for a in np.arange(-45.0, 45.01, 5.0):
        q = (Rot(0, -a, 0) * Pos(plug)).position
        tilt.append((float(a), (q - tie).length))
    edge = Vector(-SHELF_R1, 0, shelf_top)
    post = Vector(POST_X1, 0, POST_TOP)
    pan = []
    for a in np.arange(-90.0, 90.01, 15.0):
        q = (Rot(0, 0, a) * Pos(edge)).position
        pan.append((float(a), (q - post).length))
    return {"tilt": tilt, "pan": pan}


def main() -> None:
    out = HERE  # print/: these files are committed and ordered
    for name, part in printed_parts().items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(
            part, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1
        )
        print(
            f"{name}: {part.volume / 1000:.2f} cm3, bbox {part.bounding_box().size}, valid {part.is_valid}"
        )
    c, m = panned_com()
    tc, tm = tilting_com()
    print(f"lens height above chassis: {lens_height():.1f} mm")
    print(
        f"panned mass {m:.1f} g, CoM (lens frame) x {c.X:.2f} y {c.Y:.2f} z {c.Z:.2f}"
    )
    print(
        f"pan axis at x {-PAN_AXIS_BEHIND_LENS:.2f}: CoM offset x {c.X + PAN_AXIS_BEHIND_LENS:+.2f}, y {c.Y:+.2f} mm"
    )
    print(f"tilting mass {tm:.1f} g, CoM from tilt axis x {tc.X:.2f} z {tc.Z:.2f} mm")
    sp = ribbon_spans()
    for k in ("tilt", "pan"):
        ls = [v for _, v in sp[k]]
        print(f"ribbon {k} span chord: {min(ls):.1f} .. {max(ls):.1f} mm")
    if abs(c.X + PAN_AXIS_BEHIND_LENS) > 0.5:
        print("WARNING: set PAN_AXIS_BEHIND_LENS to", round(-c.X, 1))


if __name__ == "__main__":
    main()
