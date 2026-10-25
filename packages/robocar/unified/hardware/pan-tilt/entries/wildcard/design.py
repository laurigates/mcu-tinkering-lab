"""Majakka ("lighthouse"): a pan-tilt where neither servo moves.

Tournament entry ``wildcard`` for robocar-unified head 1 on two MG90D servos.

Every other pan-tilt carries the tilt servo on the pan stage, so the tilt
servo's 13 g and its 25 cm lead swing with every pan move. Here both MG90Ds
are screwed into a fixed base, side by side, and only the head turns:

* **Pan** goes through a pair of printed spur gears (1:1, module 1.25, 20
  teeth). The pan servo sits off-axis; its pinion drives a gear on the turret's
  hub, and the hub runs in two 6801ZZ ball bearings in the base. The pan servo's
  spline sees only the tooth force, never the head's weight or tipping moment.
* **Tilt** goes up the pan axis. The hub is hollow (a 4.4 mm square bore) and a
  square plunger slides in it. At the bottom the plunger is screwed into a
  grooved collar; a lever on the tilt servo's horn holds one steel dowel in that
  groove, like the shift fork on a gearbox collar. The collar turns freely over
  the dowel, so panning never moves the tilt, and the tilt servo stays where it
  is. (Revision 2: the lever replaced a two-sided fork that could not be
  assembled; see INSPECTION.md, "Revision".) At the top a bracket on the plunger pushes a
  short link to a crank on the head cradle. The cradle swings on two M3 pivots
  in the turret's cheeks, so the head is carried on both sides and no servo
  shaft carries it.

The only cable that crosses the pan joint is the head's 14-way ribbon. Both
servo leads stay still inside the base.

Frames: world as in common/fixtures.py (+X forward, +Y left, +Z up, mm), with
z 0 the top of the chassis plate and x = y = 0 the pan axis. The head is built
in its own frame (fixtures.head()) and placed with ``head_to_world``.
``printed_parts()`` returns every printed part in its WORLD pose at pan 0,
tilt 0. Print orientation is free for MJF (no supports); the README lists the
preferred one per part.

Run ``python design.py`` to write output/<part>.step and .stl and print the
tilt servo's angle table.
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
    Cone,
    Cylinder,
    Face,
    Location,
    Part,
    Pos,
    Rot,
    Wire,
    export_step,
    export_stl,
    extrude,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))  # hardware/pan-tilt, for common.*

from common import fixtures as fx  # noqa: E402
from common.check import Body  # noqa: E402

# ---------------------------------------------------------------------------
# Parameters. The first block is what Lauri must caliper before ordering (see
# the README's "Measure first"). Everything below re-derives from them.
# ---------------------------------------------------------------------------
SERVO = fx.MG90D  # E tab span, F below-tab depth, shaft offset: fixtures.py
FIT = 0.3  # servo case to cavity, per side (MJF +-0.3 mm)
GAP = 0.5  # running clearance between parts that move relative to each other

# Tilt chain. CRANK_R is the cradle crank, FORK_R the fork's dowel radius on the
# tilt horn; LINK_L the link's hole spacing. Tilt +45..-45 maps to the servo
# angles that tilt_to_servo() prints (about +-35 deg, not 1:1).
CRANK_R = 9.0
FORK_R = 11.0
LINK_L = 10.0
TILT_MAX = 46.0  # clearances are built for +-this (check sweeps +-45)

# Head pose in the head's own frame: the tilt axis is on the board's back face
# (x 0), AXIS_Z above its bottom edge. HEAD_Y shifts the head to -Y so its J2
# ribbon keep-out (head y -15.5..2.5) clears the plunger on the pan axis.
AXIS_X = 0.0
AXIS_Z = 8.0
HEAD_Y = -6.5

# Gears: 1:1, but the turret turns the opposite way to the pan servo. The
# console has no invert option: servo_controller.c must negate the pan angle
# (and look up the tilt angle, tilt_to_servo()) before it writes the PCA9685.
GEAR_M = 1.25
GEAR_Z = 20
GEAR_FACE = 5.0  # the pinion
TURRET_GEAR_FACE = 7.0  # the turret gear: 1 mm wider than the pinion each side
PINION_Z_CLEAR = 1.5  # pinion to the deck below and to the hood above
GEAR_BACKLASH = 0.3  # tooth thinning at the pitch circle, mm of arc (both gears)
# The pan servo is pushed against a printed datum face on its -Y side (toward
# the turret) by a jack screw through the +Y wall; DATUM_GAP is the nominal
# gap between the case and that face.
DATUM_GAP = 0.05
HOOD_T = 1.5  # gear hood: top plate and skirt

# Stock parts.
BRG_D, BRG_OD, BRG_W = 12.0, 21.0, 5.0  # 6801ZZ (12 x 21 x 5), two of them
HUB_D = 11.85  # under the 12.0 bore: sand to a slide fit, then retaining compound
BRG_INNER_R = 7.0  # inner ring shoulder radius the hub boss bears on
ECLIP_GROOVE_D = 9.0  # DIN 6799 size 9 E-clip on the 12 mm hub
ECLIP_GROOVE_W = 1.1
ECLIP_OD, ECLIP_T = 17.0, 0.9
PLUNGER_SQ = 4.0  # square plunger section
SQ_FIT = 0.2  # per side, plunger in the hub's square bore
DOWEL_D, DOWEL_L = 3.0, 8.0  # 1x ISO 2338 3m6 x 8 steel dowel (the lever's pin)
HORN_TRIM_R = 8.0  # double horn, both arms cut to 8 mm: clears the dowel boss
# Head board standoff: the XIAO's through-hole socket pins stand 1-2 mm proud
# of the board's back face, so the cradle's back plate sits STANDOFF behind it
# and the board rests on four pads (two M2 bosses, two plain pads).
STANDOFF = 2.0

# ---------------------------------------------------------------------------
MIN, CEN, MAX = Align.MIN, Align.CENTER, Align.MAX

# Plunger planes (world y): crank on the cradle, then the link, then the
# bracket's tab. The J2 keep-out ends at head y 2.5 = world HEAD_Y + 2.5.
Y_CRANK = (HEAD_Y + 3.5, HEAD_Y + 6.0)
Y_LINK = (Y_CRANK[1] + 0.5, Y_CRANK[1] + 3.0)
Y_TAB = (Y_LINK[1] + 0.5, Y_LINK[1] + 3.0)
LINK_END_R = 2.5
TAB_R = 3.0

# Collar on the plunger: two flanges round a groove the dowels run in.
COLLAR_R = 7.0
COLLAR_CORE_R = 4.2
GROOVE_W = DOWEL_D + 0.4
FLANGE_T = 1.5
COLLAR_HALF = GROOVE_W / 2 + FLANGE_T

# Tilt servo lies on its side, shaft along +X, FORK_R to the -Y of the pan axis.
TILT_SHAFT_Y = -FORK_R
TILT_CASE_TOP_X = -15.0  # its case top (spline side) faces +X here
FORK_PLATE_T = 3.0

# Pan servo upright, shaft at (0, PAN_Y): the gear centre distance.
PAN_Y = GEAR_M * GEAR_Z  # 1:1, so centre distance = m z

FLOOR_T = 2.5
PIT_Y1 = 16.3  # floor pit's +Y edge (collar insertion), short of the datum wall
JACK_Z = 12.0  # height of the pan servo's jack screw
EAR_W = 9.0
WALL_T = 2.5
DECK_T = 2.0


# --- small helpers ------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(MIN, MIN, MIN))


def _cyl_z(r, z0, z1, x=0.0, y=0.0) -> Part:
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(CEN, CEN, MIN))


def _cyl_x(r, x0, x1, y=0.0, z=0.0) -> Part:
    return Location((x0, y, z), (0, 90, 0)) * Cylinder(
        r, x1 - x0, align=(CEN, CEN, MIN)
    )


def _cyl_y(r, y0, y1, x=0.0, z=0.0) -> Part:
    return Location((x, y0, z), (-90, 0, 0)) * Cylinder(
        r, y1 - y0, align=(CEN, CEN, MIN)
    )


def _slot_x(x0, x1, ya, za, yb, zb, r) -> Part:
    """A stadium in the YZ plane from (ya, za) to (yb, zb), thick x0..x1."""
    length = math.hypot(yb - ya, zb - za)
    ang = math.degrees(math.atan2(zb - za, yb - ya))
    bar = Pos(0, length / 2, 0) * Box(x1 - x0, length, 2 * r, align=(MIN, CEN, CEN))
    s = Location((x0, ya, za), (ang, 0, 0)) * bar
    return s + _cyl_x(r, x0, x1, ya, za) + _cyl_x(r, x0, x1, yb, zb)


def _slot_y(y0, y1, xa, za, xb, zb, r) -> Part:
    """A stadium in the XZ plane from (xa, za) to (xb, zb), thick y0..y1."""
    length = math.hypot(xb - xa, zb - za)
    ang = math.degrees(math.atan2(zb - za, xb - xa))
    bar = Pos(length / 2, 0, 0) * Box(length, y1 - y0, 2 * r, align=(CEN, MIN, CEN))
    s = Location((xa, y0, za), (0, -ang, 0)) * bar
    return s + _cyl_y(r, y0, y1, xa, za) + _cyl_y(r, y0, y1, xb, zb)


def _gear_profile(z: int, m: float, backlash: float) -> list[tuple[float, float]]:
    """Involute spur gear outline, tooth 0 centred on +X. Tip cut 0.15 m short
    so the tip land is wide enough for MJF."""
    alpha = math.radians(20)
    rp = m * z / 2
    rb = rp * math.cos(alpha)
    ra = rp + 0.85 * m
    rf = rp - 1.25 * m

    def inv(a):
        return math.tan(a) - a

    half_p = math.pi / (2 * z) - backlash / (2 * rp)  # half tooth angle at rp
    pts = []
    radii = np.linspace(max(rb, rf), ra, 10)
    for k in range(z):
        c = 2 * math.pi * k / z
        flank = []
        for r in radii:
            a = math.acos(min(1.0, rb / r))
            flank.append((r, half_p + inv(alpha) - inv(a)))
        # root on the leading side of this tooth's space, then up the flanks
        a_root = flank[0][1]
        left = [(rf, c - a_root - 0.25 * math.pi / z)]
        left += [(r, c - th) for r, th in flank]
        right = [(r, c + th) for r, th in reversed(flank)]
        right += [(rf, c + a_root + 0.25 * math.pi / z)]
        for r, th in left + right:
            pts.append((r * math.cos(th), r * math.sin(th)))
    return pts


def gear(z: int = GEAR_Z, m: float = GEAR_M, face: float = GEAR_FACE) -> Part:
    pts = _gear_profile(z, m, GEAR_BACKLASH)
    wire = Wire.make_polygon([(x, y, 0) for x, y in pts], close=True)
    return extrude(Face(wire), face)


# --- kinematics ---------------------------------------------------------------
@functools.cache
def _layout() -> dict:
    """Heights, derived bottom-up from the parts' stack and the head sweep."""
    s45 = math.sin(math.radians(TILT_MAX))
    c45 = math.cos(math.radians(TILT_MAX))
    drop0 = LINK_L  # link vertical at tilt 0
    lean = CRANK_R * (1 - c45)
    drop_max = math.sqrt(LINK_L**2 - lean**2)
    dz_up = CRANK_R * s45 + (drop0 - drop_max)  # plunger rise, nose down
    dz_down = CRANK_R * s45 - (drop0 - drop_max)  # plunger fall, nose up
    L = {}
    L["collar_z0"] = 1.6 + dz_down + COLLAR_HALF  # collar bottom 1.6 above the plate
    L["hub_z0"] = L["collar_z0"] + dz_up + COLLAR_HALF + 0.4
    # E-clip groove under the lower bearing's inner ring
    L["groove"] = (L["hub_z0"] + 1.4, L["hub_z0"] + 1.4 + ECLIP_GROOVE_W)
    L["brg1"] = (L["groove"][1], L["groove"][1] + BRG_W)
    L["lip"] = (L["brg1"][1], L["brg1"][1] + 1.5)
    L["brg2"] = (L["lip"][1], L["lip"][1] + BRG_W)
    L["housing_top"] = L["brg2"][1]
    # turret gear: its height is set by the thrust boss on the inner ring
    L["gear_z0"] = L["housing_top"] + 0.5
    L["gear_z1"] = L["gear_z0"] + TURRET_GEAR_FACE
    # pinion: its height comes from the pan servo's stack (lintel, F, C, horn
    # seat), all unmeasured, so it gets PINION_Z_CLEAR above and below and sits
    # inside the wider turret gear's face with 1 mm to spare each side
    L["pinion_z0"] = L["housing_top"] + PINION_Z_CLEAR
    L["pinion_z1"] = L["pinion_z0"] + GEAR_FACE
    L["pan_top"] = L["pinion_z0"] - 4.5  # the servo-frame z of the pan servo
    L["deck"] = (L["housing_top"] - DECK_T, L["housing_top"])
    L["hood"] = (
        L["pinion_z1"] + PINION_Z_CLEAR,
        L["pinion_z1"] + PINION_Z_CLEAR + HOOD_T,
    )
    L["floor_z0"] = max(L["hood"][1], L["gear_z1"]) + 0.8  # turret floor over the hood
    L["floor_z1"] = L["floor_z0"] + 2.5
    # J2 keep-out's lowest point over +-TILT_MAX, from the head frame
    jx0 = -fx.J2_KEEPOUT[0] - AXIS_X
    jz0 = fx.J2_CENTRE_YZ[1] - fx.J2_KEEPOUT[2] / 2 - AXIS_Z
    j2_low = min(
        jx0 * math.sin(math.radians(t)) + jz0 * math.cos(math.radians(t))
        for t in np.linspace(-TILT_MAX, TILT_MAX, 93)
    )
    L["za"] = L["floor_z1"] + GAP - j2_low
    L["xa"] = CRANK_R + 2.5  # P sits CRANK_R behind the axis, 2.5 ahead of the plunger
    L["px"] = L["xa"] - CRANK_R
    L["dz_up"], L["dz_down"] = dz_up, dz_down
    return L


def crank_tip(tilt: float) -> tuple[float, float]:
    """World (x, z) of the crank's link pin at a tilt (positive = look up)."""
    L = _layout()
    t = math.radians(tilt)
    return L["xa"] - CRANK_R * math.cos(t), L["za"] - CRANK_R * math.sin(t)


def link_low_pin(tilt: float) -> float:
    """World z of the link's lower pin, on the plunger bracket (x fixed)."""
    L = _layout()
    tx, tz = crank_tip(tilt)
    return tz - math.sqrt(LINK_L**2 - (tx - L["px"]) ** 2)


def plunger_dz(tilt: float) -> float:
    return link_low_pin(tilt) - link_low_pin(0.0)


def tilt_to_servo(tilt: float) -> float:
    """Tilt servo angle (deg, about +X world) for a head tilt (deg)."""
    return math.degrees(math.asin(plunger_dz(tilt) / FORK_R))


# --- stand-in placement -------------------------------------------------------
def pan_servo_loc() -> Location:
    return Location((0, PAN_Y, _layout()["pan_top"]))


def tilt_servo_loc() -> Location:
    # servo frame: Z (shaft) -> +X, X (long axis) -> -Y, Y -> -Z
    return (
        Pos(TILT_CASE_TOP_X, TILT_SHAFT_Y, _layout()["collar_z0"])
        * Rot(-90, 0, 0)
        * Rot(0, 90, 0)
    )


def head_to_world(tilt: float) -> Location:
    L = _layout()
    return Pos(L["xa"], HEAD_Y, L["za"]) * Rot(0, -tilt, 0) * Pos(-AXIS_X, 0, -AXIS_Z)


def fork_rot(tilt: float) -> Location:
    L = _layout()
    phi = tilt_to_servo(tilt)
    zc = L["collar_z0"]
    return Pos(0, TILT_SHAFT_Y, zc) * Rot(phi, 0, 0) * Pos(0, -TILT_SHAFT_Y, -zc)


def trimmed_horn() -> Part:
    """fixtures double horn, both arms cut to HORN_TRIM_R (servo frame)."""
    h = fx.horn("double", SERVO)
    return h & Cylinder(HORN_TRIM_R, 20, align=(CEN, CEN, MIN))


# --- printed parts (world pose, pan 0, tilt 0) -----------------------------------
def _sweep_bbox(make, tilts) -> tuple[float, ...]:
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for t in tilts:
        bb = make(t).bounding_box()
        lo = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z])
        hi = np.maximum(hi, [bb.max.X, bb.max.Y, bb.max.Z])
    return (*lo, *hi)


def link_part(tilt: float = 0.0) -> Part:
    L = _layout()
    tx, tz = crank_tip(tilt)
    pz = link_low_pin(tilt)
    bar = _slot_y(*Y_LINK, L["px"], pz, tx, tz, LINK_END_R)
    holes = _cyl_y(1.15, Y_LINK[0] - 1, Y_LINK[1] + 1, L["px"], pz) + _cyl_y(
        1.15, Y_LINK[0] - 1, Y_LINK[1] + 1, tx, tz
    )
    return bar - holes


def bracket_part(dz: float = 0.0) -> Part:
    """Plunger-top bracket: a socket on the rod and a tab carrying the link pin."""
    L = _layout()
    pz = link_low_pin(0.0) + dz
    blk_top = pz - LINK_END_R - GAP
    blk_bot = blk_top - 5.0
    half = PLUNGER_SQ / 2 + SQ_FIT + 1.3
    block = _box(-half, half, -half, Y_TAB[1], blk_bot, blk_top)
    tab = _box(-half, L["px"], *Y_TAB, blk_bot, pz) + _cyl_y(TAB_R, *Y_TAB, L["px"], pz)
    b = block + tab
    sq = PLUNGER_SQ + 2 * SQ_FIT
    b -= _box(-sq / 2, sq / 2, -sq / 2, sq / 2, blk_bot - 1, blk_bot + 4.0)
    b -= _cyl_z(0.9, blk_bot, blk_top + 1)  # M2 screw down into the rod's end
    b -= _cyl_y(0.8, Y_TAB[0] - 0.1, Y_TAB[1] + 0.1, L["px"], pz)  # M2 pilot
    return b


COLLAR_SOCKET = 3.5  # depth of the rod's square socket in the collar's top
COLLAR_CBORE = 1.5  # counterbore for the M2 pan head under the collar


def rod_bottom(dz: float = 0.0) -> float:
    return _layout()["collar_z0"] + dz + COLLAR_HALF - COLLAR_SOCKET


def plunger_part(dz: float = 0.0) -> Part:
    """The square rod. It goes DOWN through the hub from above, after the
    turret, into the collar's socket; screws at both ends."""
    rod_top = link_low_pin(0.0) + dz - LINK_END_R - GAP - 5.0 + 4.0  # into the socket
    z0 = rod_bottom(dz)
    s = PLUNGER_SQ
    p = _box(-s / 2, s / 2, -s / 2, s / 2, z0, rod_top)
    p -= _cyl_z(0.8, rod_top - 5, rod_top + 0.1)  # M2 pilot for the bracket screw
    p -= _cyl_z(0.8, z0 - 0.1, z0 + 5.0)  # M2 pilot for the collar screw
    return p


def collar_part(dz: float = 0.0) -> Part:
    """Grooved collar. Slides in sideways (along -Y) over the lever's dowel from
    below, through the floor pit; the rod then drops into its socket."""
    zc = _layout()["collar_z0"] + dz
    z0, z1 = zc - COLLAR_HALF, zc + COLLAR_HALF
    core = _cyl_z(COLLAR_CORE_R, z0, z1)
    f1 = _cyl_z(COLLAR_R, z0, zc - GROOVE_W / 2)
    f2 = _cyl_z(COLLAR_R, zc + GROOVE_W / 2, z1)
    c = core + f1 + f2
    sq = PLUNGER_SQ + 2 * 0.1
    c -= _box(-sq / 2, sq / 2, -sq / 2, sq / 2, z1 - COLLAR_SOCKET, z1 + 1)
    c -= _cyl_z(1.1, z0 - 1, z1)  # M2 clearance
    c -= _cyl_z(2.1, z0 - 1, z0 + COLLAR_CBORE)  # pan-head counterbore
    return c


# Head-frame (y, z) of the two plain rest pads: outboard of the XIAO's socket
# rows (y 0.5 +- 7.62) and above J2's keep-out (z < 7.5).
PADS_YZ = [(12.5, 13.0), (-12.5, 13.0)]


def cradle_part() -> Part:
    """In the HEAD frame (board back face x 0, bottom edge z 0)."""
    hs = fx.HEAD_SIZE / 2
    arm0, arm1 = hs + 0.5, hs + 3.0
    xb1 = -STANDOFF  # the back plate's front face, clear of the THT pins
    xb0 = xb1 - 3.0
    back = _box(xb0, xb1, -arm1, arm1, 8.0, fx.HEAD_SIZE)
    c = back
    for y, z in PADS_YZ:  # plain rest pads, clear of both XIAO pin rows
        c += _cyl_x(2.0, xb0, 0.0, y, z)
    for sgn in (1, -1):
        ya, yb = sorted((sgn * arm0, sgn * arm1))
        c += _box(xb0, AXIS_X + 4.0, ya, yb, AXIS_Z - 4.0, fx.HEAD_SIZE)
        boss_a, boss_b = sorted((sgn * arm0, sgn * (arm1 + 1.5)))
        c += _cyl_y(4.0, boss_a, boss_b, AXIS_X, AXIS_Z)
    # board screw bosses: R3 in the plate, R2.6 standoffs in front of it (the
    # +Y socket row's top pin is 3.7 mm from the hole), M2 inserts from behind
    for y, z in fx.HEAD_HOLES_YZ:
        c += _cyl_x(3.0, -5.5, xb1, y, z) + _cyl_x(2.6, xb1 - 0.1, 0.0, y, z)
    # crank, behind the board in the plunger's plane, outside J2's band
    yc = (Y_CRANK[0] - HEAD_Y, Y_CRANK[1] - HEAD_Y)
    c += _slot_y(*yc, -3.0 + 2.0, AXIS_Z + 1.0, AXIS_X - CRANK_R, AXIS_Z, 3.0)
    c += _box(xb0, -1.0, *yc, AXIS_Z - 2.0, 10.0)
    # holes
    for y, z in fx.HEAD_HOLES_YZ:
        c -= _cyl_x(1.6, -5.6, xb1, y, z)  # M2 insert, 3.2 bore, from the back
        c -= _cyl_x(1.1, xb1 - 0.1, 0.1, y, z)  # M2 clearance through the standoff
    for sgn in (1, -1):
        outer = sgn * (arm1 + 1.5)
        ya, yb = sorted((outer + sgn * 1.0, outer - sgn * 5.0))
        c -= _cyl_y(2.0, ya, yb, AXIS_X, AXIS_Z)  # M3 insert, 4.0 bore
    c -= _cyl_y(0.8, yc[0] - 0.1, yc[1] + 0.1, AXIS_X - CRANK_R, AXIS_Z)  # M2 pilot
    return c


CHEEK_IN = fx.HEAD_SIZE / 2 + 3.0 + 1.5 + 0.5  # head-frame |y| of a cheek's inner face
CHEEK_T = 3.0


def turret_part() -> Part:
    L = _layout()
    xa, za = L["xa"], L["za"]
    z0f, z1f = L["floor_z0"], L["floor_z1"]
    hub = _cyl_z(HUB_D / 2, L["hub_z0"], L["gear_z0"])
    boss = _cyl_z(BRG_INNER_R, L["housing_top"] + 0.0, L["gear_z0"])
    g = Pos(0, 0, L["gear_z0"]) * Rot(0, 0, 90) * gear(face=TURRET_GEAR_FACE)
    neck = _cyl_z(10.5, L["gear_z1"], z0f + 0.1)
    plate = _box(
        -6.0,
        xa + 8.0,
        HEAD_Y - CHEEK_IN - CHEEK_T,
        HEAD_Y + CHEEK_IN + CHEEK_T,
        z0f,
        z1f,
    )
    plate += _cyl_z(10.0, z0f, z1f)
    t = hub + boss + g + neck + plate
    for sgn in (1, -1):
        y_in = HEAD_Y + sgn * CHEEK_IN
        ya, yb = sorted((y_in, y_in + sgn * CHEEK_T))
        cheek = _box(xa - 6.0, xa + 6.0, ya, yb, z0f, za) + _cyl_y(6.0, ya, yb, xa, za)
        cheek -= _cyl_y(1.65, ya - 1, yb + 1, xa, za)  # M3 journal
        t += cheek
        # gusset in front of the cheek, down to the plate
        t += _box(xa + 6.0, xa + 8.0, ya, yb, z0f, z1f + 8.0)
    sq = PLUNGER_SQ + 2 * SQ_FIT
    t -= _box(-sq / 2, sq / 2, -sq / 2, sq / 2, L["hub_z0"] - 1, z1f + 1)
    # room for the bracket and the link over the whole tilt
    tilts = np.linspace(-TILT_MAX, TILT_MAX, 9)
    for make in (
        lambda tt: bracket_part(plunger_dz(tt)),
        lambda tt: link_part(tt),
    ):
        x0, y0, zz0, x1, y1, zz1 = _sweep_bbox(make, tilts)
        t -= _box(x0 - GAP, x1 + GAP, y0 - GAP, y1 + GAP, zz0 - GAP, zz1 + 20)
    # E-clip groove: the clip holds the hub down against the lower inner ring
    t -= _cyl_z(BRG_D / 2 + 1, *L["groove"]) - _cyl_z(
        ECLIP_GROOVE_D / 2, L["groove"][0] - 1, L["groove"][1] + 1
    )
    return t


def pinion_part() -> Part:
    L = _layout()
    g = gear()
    g = Rot(0, 0, -90 + 180 / GEAR_Z) * g  # a space faces the turret gear's tooth
    g -= Cylinder(2.6, 20, align=(CEN, CEN, CEN))  # horn screw and screwdriver
    for a in (0, 120, 240):  # three M2 x 5 self-tappers up through the disc horn
        g -= (
            Rot(0, 0, a)
            * Pos(7.0, 0, 0)
            * Cylinder(0.8, 3.5 * 2, align=(CEN, CEN, CEN))
        )
    return Pos(0, PAN_Y, L["pinion_z0"]) * g


LEVER_X0 = TILT_CASE_TOP_X + 4.5  # the lever plate's back face, on the horn's top
LEVER_BOSS_X0 = LEVER_X0 - 2.0  # dowel boss behind the plate, past the cut arm tip
DOWEL_X0 = LEVER_X0 + FORK_PLATE_T + 3.0 - DOWEL_L  # dowel tip 3 mm into the groove


def lever_part() -> Part:
    """One plate on the tilt horn, one dowel in the collar groove. No far side:
    the spline carries the (small) plunger force, nothing is coaxial with it."""
    L = _layout()
    zc = L["collar_z0"]
    ys = TILT_SHAFT_Y
    xh0 = LEVER_X0
    xh1 = xh0 + FORK_PLATE_T
    bar_end = ys - 6.0
    f = _slot_x(xh0, xh1, bar_end, zc, 0.0, zc, 3.5)
    f += _cyl_x(2.6, LEVER_BOSS_X0, xh0 + 0.1, 0.0, zc)  # dowel boss
    f -= _cyl_x(2.3, xh0 - 1, xh1 + 1, ys, zc)  # horn screw head and driver
    f -= _cyl_x(DOWEL_D / 2 - 0.05, LEVER_BOSS_X0 - 1, xh1 + 1, 0.0, zc)  # press fit
    for dy in (-6.0, 6.0):  # two M2 self-tappers into the trimmed horn's arms
        f -= _cyl_x(1.15, xh0 - 0.1, xh1 + 0.1, ys + dy, zc)
    return f


HOOD_R = 15.0  # inner radius of the hood's skirt, about the pinion axis
HOOD_SCREWS = [(16.0, 14.5), (-16.5, 14.5)]  # M2 into bosses under the deck


def hood_part() -> Part:
    """Fixed guard over the pinion and the mesh. The turret gear is below its
    top plate; the turret plate sweeps over it."""
    L = _layout()
    top = L["housing_top"]
    h0, h1 = L["hood"]
    # lid: over the pinion and the mesh point (0, 12.5), clear of the turret
    # neck (R10.5); squared off at y 10 so its edge has no knife-edge cusps
    lid = _cyl_z(HOOD_R + HOOD_T, h0, h1, 0, PAN_Y) - _cyl_z(11.5, h0 - 1, h1 + 1)
    lid -= _box(-30, 30, PAN_Y - 30, 10.0, h0 - 1, h1 + 1)
    skirt = _cyl_z(HOOD_R + HOOD_T, top, h1, 0, PAN_Y) - _cyl_z(
        HOOD_R, top - 1, h1 + 1, 0, PAN_Y
    )
    # open where the turret gear (tip R13.6) runs through: a sector about the
    # pinion axis, so the skirt's ends are radial. Inner corners at y 12.8,
    # 1.4 mm or more off the gear's tips.
    th = math.acos((PAN_Y - 12.8) / HOOD_R)
    R = 40.0
    tri = Wire.make_polygon(
        [
            (0, PAN_Y, 0),
            (-R * math.sin(th), PAN_Y - R * math.cos(th), 0),
            (R * math.sin(th), PAN_Y - R * math.cos(th), 0),
        ],
        close=True,
    )
    sector = Pos(0, 0, top - 1) * extrude(Face(tri), h1 - top + 2)
    assert sector.bounding_box().min.Z >= top - 1.01  # extruded upward
    skirt -= sector
    # Each end gets a straight 3 mm block along the skirt's tangent. A curved
    # wall measured square off its end face reads as a sliver near the outer
    # edge (the ray leaves the curve tangentially); a straight end does not.
    rm = HOOD_R + HOOD_T / 2
    for sgn in (1, -1):
        phi = -math.pi / 2 + sgn * th  # end angle about the pinion axis
        ang = math.degrees(phi) + 90.0 * sgn  # tangent, pointing into the skirt
        blk = Box(3.0, HOOD_T + 0.2, h1 - top, align=(MIN, CEN, MIN))
        skirt += (
            Location((rm * math.cos(phi), PAN_Y + rm * math.sin(phi), top), (0, 0, ang))
            * blk
        )
    # 0.3 mm chamfer on the skirt's outer bottom edge (seats on the deck past
    # an MJF edge bead)
    cone = Pos(0, PAN_Y, top) * Cone(
        HOOD_R + HOOD_T - 0.3, HOOD_R + HOOD_T, 0.3, align=(CEN, CEN, MIN)
    )
    skirt -= _cyl_z(HOOD_R + HOOD_T + 1, top - 0.01, top + 0.3, 0, PAN_Y) - cone
    h = lid + skirt
    for x, y in HOOD_SCREWS:
        ear = _cyl_z(3.0, top, top + 2.5, x, y)
        # bar from the ear to the skirt, toward the pinion axis
        ang = math.degrees(math.atan2(PAN_Y - y, -x))
        d = math.hypot(x, PAN_Y - y)
        bar = Location((x, y, top), (0, 0, ang)) * Box(
            d - HOOD_R - 0.5, 4.0, 2.5, align=(MIN, CEN, MIN)
        )
        h += ear + bar
        h -= _cyl_z(1.1, top - 1, top + 3, x, y)
    return h


def base_part() -> Part:
    L = _layout()
    zc = L["collar_z0"]
    x0, x1 = -47.0, 19.0
    y0, y1 = -27.0, PAN_Y + 6.0 + FIT + WALL_T + 0.5
    top = L["housing_top"]
    # floor with two chassis ears (4 x M3)
    b = _box(x0, x1, y0, y1, 0, FLOOR_T)
    for x in (x0 - EAR_W, x1):
        b += _box(x, x + EAR_W, y0, y1, 0, FLOOR_T)
    # perimeter wall
    shell = _box(x0, x1, y0, y1, 0, top) - _box(
        x0 + WALL_T, x1 - WALL_T, y0 + WALL_T, y1 - WALL_T, -1, top + 1
    )
    b += shell
    # deck and bearing housing
    b += _box(x0, x1, y0, y1, *L["deck"])
    b += _cyl_z(BRG_OD / 2 + 2.0, L["brg1"][0], top)
    b -= _cyl_z(BRG_OD / 2, L["brg1"][0] - 1, L["lip"][0])
    b -= _cyl_z(BRG_OD / 2, L["lip"][1], top + 1)
    b -= _cyl_z(BRG_OD / 2 - 2.0, L["lip"][0] - 1, L["lip"][1] + 1)
    # pan servo: drop-in opening, tab lintels, floor hole
    pc = -SERVO.length / 2 + SERVO.shaft_from_end  # case centre x
    tab_half = SERVO.tab_span / 2
    tab_z = L["pan_top"] - (SERVO.height - SERVO.tab_underside)
    for xa_, xb_ in (
        (pc - tab_half - 1.5, pc - SERVO.length / 2 - FIT),
        (pc + SERVO.length / 2 + FIT, pc + tab_half + 1.5),
    ):
        b += _box(xa_, xb_, PAN_Y - 7.0, y1 - WALL_T, tab_z - 8.0, tab_z)
    for xa_, xb_ in (
        (pc - tab_half - 3.0, pc - tab_half - 1.0),
        (pc + tab_half + 1.0, pc + tab_half + 3.0),
    ):
        b += _box(xa_, xb_, PAN_Y - 7.0, y1 - WALL_T, tab_z - 8.0, top)
    b -= _box(
        pc - tab_half - 0.6,
        pc + tab_half + 0.6,
        PAN_Y - 6.0 - FIT - 0.4,
        PAN_Y + 6.0 + FIT + 0.4,
        tab_z,
        top + 1,
    )
    b -= _cyl_z(10.5 + 1.0, L["deck"][0] - 1, top + 1, 0, PAN_Y)
    b -= _box(-11.5, 11.5, PAN_Y, y1 + 1, L["deck"][0] - 1, top + 1)  # pinion notch
    b -= _box(
        pc - SERVO.length / 2 - FIT,
        pc + SERVO.length / 2 + FIT,
        PAN_Y - 6.0 - FIT,
        PAN_Y + 6.0 + FIT,
        -1,
        tab_z,
    )
    # the lead's boot, under the -X lintel
    b -= _box(
        pc - SERVO.length / 2 - 7.0,
        pc - SERVO.length / 2,
        PAN_Y - 3.0,
        PAN_Y + 3.0,
        -1,
        tab_z - 8.5,
    )
    for hx, _, _ in fx.servo_tab_holes(SERVO):
        b -= _cyl_z(0.8, tab_z - 6.0, tab_z + 0.1, hx, PAN_Y)
    # Y datum: the gear centre distance is the turret axis to this face, plus
    # half the case width. A jack screw through the +Y wall holds the case on it.
    dy1 = PAN_Y - SERVO.width / 2 - DATUM_GAP
    b += _box(
        pc - SERVO.length / 2 + 1.0,
        pc + SERVO.length / 2 - 1.0,
        dy1 - 2.0,
        dy1,
        0,
        tab_z - 1.0,
    )
    b -= _cyl_y(0.8, y1 - WALL_T - 0.1, y1 + 0.1, pc, JACK_Z)
    # tilt servo: bulkhead with an open-topped window, deck opening above it
    bx1 = TILT_CASE_TOP_X - (SERVO.height - SERVO.tab_underside)
    bx0 = bx1 - 3.0
    tw = SERVO.width / 2 + FIT
    yb0 = TILT_SHAFT_Y - 6.0 - FIT
    yb1 = TILT_SHAFT_Y + (SERVO.length - SERVO.shaft_from_end) + FIT
    bulk = _box(bx0, bx1, y0 + WALL_T, TILT_SHAFT_Y + tab_half + 6.0, 0, zc + tw + 4.0)
    bulk -= _box(bx0 - 1, bx1 + 1, yb0, yb1, zc - tw, top)
    b += bulk
    # deck opening over the whole case (to its top end at x -15) so the servo
    # drops straight in, with a notch for the spline; stays off the housing
    b -= _box(
        bx0 - 21.0,
        TILT_CASE_TOP_X + FIT + 0.3,
        y0 + WALL_T,
        yb1 + 7.0,
        L["deck"][0] - 1,
        top + 1,
    )
    b -= _box(
        TILT_CASE_TOP_X,
        TILT_CASE_TOP_X + 4.6,
        TILT_SHAFT_Y - 3.0,
        TILT_SHAFT_Y + 3.0,
        L["deck"][0] - 1,
        top + 1,
    )
    for hx, _, _ in fx.servo_tab_holes(SERVO):
        # servo frame x -> world -y
        b -= _cyl_x(0.8, bx1 - 7.0, bx1 + 0.1, TILT_SHAFT_Y - hx, zc)
    # floor pit: the lever's ends run below z 2.5, and the collar comes up
    # through it at y +9 and slides -Y onto the dowel
    b -= _box(-14.5, 14.5, y0 + WALL_T, PIT_Y1, -1, FLOOR_T + 0.01)
    # +X service window (tilt servo's +Y tab screw), a driver hole for its -Y
    # tab screw and one on the spline axis for the horn screw
    b -= _box(
        x1 - WALL_T - 1, x1 + 1, TILT_SHAFT_Y + 6.0, PAN_Y - 8.0, 3.0, L["deck"][0] - 1
    )
    tab_ys = [TILT_SHAFT_Y - hx for hx, _, _ in fx.servo_tab_holes(SERVO)]
    b -= _cyl_x(2.5, x1 - WALL_T - 1, x1 + 1, min(tab_ys), zc)
    b -= _cyl_x(3.0, x1 - WALL_T - 1, x1 + 1, TILT_SHAFT_Y, zc)
    # bosses under the deck for the hood's two M2 screws
    for hx_, hy_ in HOOD_SCREWS:
        b += _cyl_z(2.75, top - 7.5, top, hx_, hy_)
        b -= _cyl_z(0.8, top - 6.0, top + 0.1, hx_, hy_)
    # lead exits (both servo leads, out of the -X wall)
    b -= _box(x0 - 1, x0 + WALL_T + 1, -4.0, PAN_Y + 4.0, FLOOR_T, FLOOR_T + 7.0)
    # chassis holes, 3.4 for M3
    for x in (x0 - EAR_W / 2, x1 + EAR_W / 2):
        for y in (y0 + 5.0, y1 - 5.0):
            b -= _cyl_z(1.7, -1, 4, x, y)
    return b


@functools.cache
def _built() -> dict:
    return {
        "base": base_part(),
        "turret": turret_part(),
        "cradle": cradle_part(),
        "link": link_part(0.0),
        "bracket": bracket_part(0.0),
        "plunger": plunger_part(0.0),
        "collar": collar_part(0.0),
        "lever": lever_part(),
        "pinion": pinion_part(),
        "hood": hood_part(),
    }


def printed_parts() -> dict[str, Part]:
    """Every printed part in its WORLD pose at pan 0, tilt 0."""
    b = dict(_built())
    b["cradle"] = head_to_world(0.0) * b["cradle"]
    return b


COLOURS = {
    "base": "#2b2b2e",
    "turret": "#c9b037",
    "cradle": "#d9822b",
    "link": "#3a7ca5",
    "bracket": "#3a7ca5",
    "plunger": "#7a9e3d",
    "collar": "#7a9e3d",
    "lever": "#b54a68",
    "pinion": "#8a6bbf",
    "hood": "#4a4a50",
}


def _stand_ins_static() -> list[Body]:
    L = _layout()
    out = [
        Body("pan_servo", pan_servo_loc() * fx.servo(SERVO), "base"),
        Body("tilt_servo", tilt_servo_loc() * fx.servo(SERVO), "base"),
    ]
    # bearings (outer rings fixed in the base, inner on the hub: a ring stand-in)
    for i, (z0, z1) in enumerate((L["brg1"], L["brg2"])):
        ring = _cyl_z(BRG_OD / 2, z0, z1) - _cyl_z(BRG_D / 2, z0 - 1, z1 + 1)
        out.append(Body(f"bearing{i + 1}", ring, "base"))
    # pan servo jack screw, M2 x 6 pan head through the +Y wall; tip DATUM_GAP
    # off the case nominally (it is turned in until the case sits on the datum)
    pc = -SERVO.length / 2 + SERVO.shaft_from_end
    yc = PAN_Y + SERVO.width / 2 + DATUM_GAP
    js = _cyl_y(1.0, yc, yc + 6.0, pc, JACK_Z) + _cyl_y(
        1.9, yc + 6.0, yc + 7.3, pc, JACK_Z
    )
    out.append(Body("jack_screw", js, "base"))
    return out


def assembly(pan_deg: float, tilt_deg: float) -> list[Body]:
    b = _built()
    L = _layout()
    pan = Rot(0, 0, pan_deg)
    dz = plunger_dz(tilt_deg)
    hw = head_to_world(tilt_deg)
    fr = fork_rot(tilt_deg)
    out = [Body("base", b["base"], "base", printed=True, colour=COLOURS["base"])]
    out += _stand_ins_static()
    # pinion group: turns -pan about its own axis
    prot = Pos(0, PAN_Y, 0) * Rot(0, 0, -pan_deg) * Pos(0, -PAN_Y, 0)
    out.append(
        Body(
            "pinion",
            prot * b["pinion"],
            "pinion",
            printed=True,
            colour=COLOURS["pinion"],
        )
    )
    out.append(
        Body(
            "pan_horn",
            prot * pan_servo_loc() * fx.horn("disc", SERVO),
            "pinion",
            allow={"pan_servo"},
        )
    )
    # turret group
    out.append(
        Body(
            "turret",
            pan * b["turret"],
            "turret",
            printed=True,
            colour=COLOURS["turret"],
        )
    )
    g0 = L["groove"][0] + (ECLIP_GROOVE_W - ECLIP_T) / 2
    clip = _cyl_z(ECLIP_OD / 2, g0, g0 + ECLIP_T) - _cyl_z(
        ECLIP_GROOVE_D / 2, g0 - 1, g0 + 2
    )
    out.append(Body("eclip", pan * clip, "turret"))
    # gear hood (fixed, but its own group so the gate tests it against the base
    # and the servos as well as the moving parts) and its two M2 x 8 screws,
    # which self-tap into the base's pilots like every other screw here, so they
    # ride in the base's group
    out.append(Body("hood", b["hood"], "hood", printed=True, colour=COLOURS["hood"]))
    for i, (hx, hy) in enumerate(HOOD_SCREWS):
        top = L["housing_top"]
        hs_ = _cyl_z(1.0, top + 2.5 - 8.0, top + 2.5, hx, hy) + _cyl_z(
            1.9, top + 2.5, top + 3.8, hx, hy
        )
        out.append(Body(f"hood_screw{i + 1}", hs_, "base"))
    # cradle + head
    out.append(
        Body(
            "cradle",
            pan * hw * b["cradle"],
            "cradle",
            printed=True,
            colour=COLOURS["cradle"],
        )
    )
    out.append(Body("head", pan * hw * fx.head(), "cradle"))
    hs = fx.HEAD_SIZE / 2
    for sgn in (1, -1):
        y_out = sgn * (CHEEK_IN + CHEEK_T)
        ya, yb = sorted((sgn * (hs + 0.5 + 0.5), y_out))
        scr = _cyl_y(1.5, ya, yb, AXIS_X, AXIS_Z)
        hy0, hy1 = sorted((y_out, y_out + sgn * 1.65))
        scr += _cyl_y(2.85, hy0, hy1, AXIS_X, AXIS_Z)  # M3 button head 5.7 x 1.65
        out.append(
            Body(f"pivot_screw_{'l' if sgn > 0 else 'r'}", pan * hw * scr, "cradle")
        )
    tx, tz = crank_tip(tilt_deg)
    # crank pin: M2 x 6 pan head from the link's +y side into the crank
    cs = _cyl_y(1.0, Y_CRANK[0] + 1.0, Y_LINK[1], tx, tz) + _cyl_y(
        1.75, Y_LINK[1], Y_LINK[1] + 1.3, tx, tz
    )
    out.append(Body("crank_screw", pan * cs, "cradle"))
    # link
    out.append(
        Body(
            "link",
            pan * link_part(tilt_deg),
            "link",
            printed=True,
            colour=COLOURS["link"],
        )
    )
    # plunger group: plunger, bracket, the bracket's pin screw
    out.append(
        Body(
            "plunger",
            pan * plunger_part(dz),
            "plunger",
            printed=True,
            colour=COLOURS["plunger"],
        )
    )
    out.append(
        Body(
            "bracket",
            pan * bracket_part(dz),
            "plunger",
            printed=True,
            colour=COLOURS["bracket"],
        )
    )
    pz = link_low_pin(tilt_deg)
    ps = _cyl_y(1.0, Y_LINK[0], Y_TAB[1] - 0.6, L["px"], pz) + _cyl_y(
        1.75, Y_LINK[0] - 1.3, Y_LINK[0], L["px"], pz
    )
    out.append(Body("bracket_screw", pan * ps, "plunger"))
    out.append(
        Body(
            "collar",
            pan * collar_part(dz),
            "plunger",
            printed=True,
            colour=COLOURS["collar"],
        )
    )
    z0c = L["collar_z0"] + dz - COLLAR_HALF + COLLAR_CBORE
    cs2 = _cyl_z(1.0, z0c, z0c + 6.0) + _cyl_z(1.9, z0c - 1.3, z0c)
    out.append(Body("collar_screw", pan * cs2, "plunger"))
    # lever group: lever, trimmed horn, one dowel
    out.append(
        Body("lever", fr * b["lever"], "fork", printed=True, colour=COLOURS["lever"])
    )
    out.append(
        Body(
            "tilt_horn",
            fr * tilt_servo_loc() * trimmed_horn(),
            "fork",
            allow={"tilt_servo"},
        )
    )
    zc = L["collar_z0"]
    out.append(
        Body(
            "dowel",
            fr * _cyl_x(DOWEL_D / 2, DOWEL_X0, DOWEL_X0 + DOWEL_L, 0.0, zc),
            "fork",
        )
    )
    return out


def main() -> None:
    out = HERE / "output"
    out.mkdir(exist_ok=True)
    for name, part in printed_parts().items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"))
        print(f"{name:8s} {part.volume / 1000:6.2f} cm3  valid={part.is_valid}")
    L = _layout()
    print(
        f"tilt axis world x {L['xa']:.1f} z {L['za']:.1f}; lens z "
        f"{L['za'] - AXIS_Z + fx.XIAO_CENTRE_YZ[1]:.1f}"
    )
    print("tilt deg -> tilt servo deg:")
    for t in (-45, -30, -15, 0, 15, 30, 45):
        print(f"  {t:+4d} -> {tilt_to_servo(t):+6.1f}")


if __name__ == "__main__":
    main()
