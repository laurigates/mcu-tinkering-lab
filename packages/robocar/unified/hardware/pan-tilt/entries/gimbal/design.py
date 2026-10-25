"""The Gimbal: a nodal pan-tilt for robocar-unified head 1 on two MG90D servos.

Design intent, in order:

1. The tilt axis runs through the camera's entrance pupil, so tilting turns the
   view without translating it (no parallax for the ADR-025 planner).
2. The pan axis sits under the centre of mass of everything that pans, so the
   pan servo sees no static bending and the car's cornering puts no torque on it.
3. Height and swept volume are minimised around those two constraints: the
   yoke and outrigger are cut by the head's own tilt sweep (the head is
   subtracted from them at every 2 degrees of tilt), and the lens height is
   computed from that sweep, not guessed.
4. The pan thrust and the side-mounted tilt servo's overturning moment go into
   a thin-section 6705ZZ ball bearing around the pan horn, not the servo shaft.
   The tilt side opposite the servo runs in an MR128ZZ, so the tilt servo's
   output shaft is not a cantilever.

Head 1 is mounted upside down (rotated 180 degrees about the camera axis,
``HEAD_FLIPPED``): the J2 ribbon plug then sits behind the board's top edge
and swings up and back when the head looks up, instead of down into the pan
stage. That alone lowers the lens about 12 mm. The firmware turns the image
upright with the sensor's own flips (the head is an OV3660) (``set_vflip(1)`` + ``set_hmirror(1)``).
With ``HEAD_FLIPPED = False`` everything below re-derives (taller, ~83 mm).

Frames. Geometry is built in the LENS frame: origin on the tilt axis at the
entrance pupil, +X forward, +Y left, +Z up. ``WORLD`` puts the pan axis at
x = y = 0 and the chassis plate's top at z = 0 (common/fixtures.py world frame:
+X forward, +Z up, mm). ``printed_parts()`` returns each part in its WORLD pose
at pan 0, tilt 0. All parts are MJF PA12, which needs no supports, so print
orientation is free; the README lists the preferred one per part.

Run ``python design.py`` to write output/<part>.step and .stl.
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
sys.path.insert(0, str(HERE.parents[1]))  # hardware/pan-tilt, for common.*

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

FIT = 0.3  # radial/side clearance of a servo case in its window (MJF +-0.3)
FIT_END = 0.15  # servo case ends (both servos): keeps >= 1.1 mm beside the tab pilots
GAP = 0.5  # running clearance between parts that move relative to each other
SWEEP_MARGIN = 1.0  # the head/cradle proxy is grown by this before cutting
TILT_CUT = 46.0  # the yoke clears the head over +-this (check sweeps +-45)
TILT_STEP = 2.0

# Pan bearing: thin-section 6705ZZ (25 x 32 x 4) around the cross horn.
PB_ID, PB_OD, PB_W = 25.0, 32.0, 4.0
PB_INNER_RACE_OD = 27.6  # land on the inner race stays inside this
# Tilt idler bearing: MR128ZZ (8 x 12 x 3.5) in the cradle's -Y cheek.
TB_ID, TB_OD, TB_W = 8.0, 12.0, 3.5

WALL = 2.0
PILOT = 1.8  # self-tapping pilot for the servos' own ~2 mm screws (modelled)
SCREW_CORE = 1.7  # stand-in diameter of a self-tapper's shank in its pilot
INSERT_HOLE = 3.2  # M2 x 3 heat-set insert (OD 3.5 knurl) in PA12
PAN_SERVO_LIFT = 0.5  # pan servo base above the chassis plate
CHASSIS_FLANGE_T = 3.0
M3_HOLES = [(-20.0, 16.0), (-20.0, -16.0), (15.0, 16.0), (15.0, -16.0)]  # world xy

# Panned centre of mass behind the lens; derived by panned_com() below and
# frozen here so the geometry is deterministic. Re-run `python design.py`
# after a change: it prints the CoM and warns if this drifts > 0.5 mm.
PAN_AXIS_BEHIND_LENS = 8.5

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
ZS = PAN_SERVO_LIFT + SERVO.height  # pan servo case top
Z_CHAMBER = ZS + 0.3  # horn chamber floor (horn hub starts 0.5 above the case)
Z_HORN_TOP = ZS + 4.5
Z_LIP0 = ZS + 5.0
Z_SEAT0 = Z_LIP0 + WALL
Z_TOWER_TOP = Z_SEAT0 + PB_W  # bearing top flush with the tower top
Z_LAND_TOP = Z_TOWER_TOP + GAP
FLANGE_T = 4.0
Z_FLANGE_TOP = Z_LAND_TOP + FLANGE_T
TOWER_R = PB_OD / 2 + 2.8
CHAMBER_R = 16.0 + 0.8  # cross horn arms 16 mm


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
CHEEK_TAB_Z = (-8.0, 6.0) if HOLE_Z < 0 else (-6.0, 8.0)


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
    disc = np.hypot(x + PAN_AXIS_BEHIND_LENS, y) < TOWER_R
    arms = (np.abs(x) <= 9.0) & (y >= OUTRIGGER_FOOT_Y - 3.0) & (y <= UPRIGHT_Y1)
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
        bridge = _box(BRIDGE_X0, BRIDGE_X1, min(ya, yi0), max(yb, yi1), zc0, zc1)
        inner = _box(BRIDGE_X0, 0, yi0, yi1, *CHEEK_TAB_Z) + _cyl_y(8.0, yi0, yi1)
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
    c = c - _cyl_y(TB_OD / 2, IDLER_FACE_Y - 1, IDLER_FACE_Y + TB_W)
    c = c - _cyl_y(4.9, IDLER_FACE_Y, -CHEEK_IN + 1)
    return c


HORN_SCREW_D = (10.0, 16.0)  # single-arm horn holes used, from the axis (estimate)
CROSS_SCREW_R = 9.0  # cross horn holes used, from the axis (estimate)


def build_yoke() -> Part:
    zl = W
    px = -PAN_AXIS_BEHIND_LENS
    spigot = _cyl_z(PB_ID / 2 - 0.05, zl(Z_HORN_TOP), zl(Z_TOWER_TOP), x=px)
    land = _cyl_z(PB_INNER_RACE_OD / 2 - 0.3, zl(Z_TOWER_TOP), zl(Z_LAND_TOP), x=px)
    zf0, zf1 = zl(Z_LAND_TOP), zl(Z_FLANGE_TOP)
    flange = _cyl_z(TOWER_R, zf0, zf1, x=px)
    arm_p = _box(-9, 9, 0, UPRIGHT_Y1, zf0, zf1)
    arm_n = _box(-9, 9, OUTRIGGER_FOOT_Y - 3.3, 0, zf0, zf1)
    upright = _box(-9, 9, UPRIGHT_Y0, UPRIGHT_Y1, zf0, 11.5)
    # gusset between flange arm and upright
    gusset = _box(
        -9,
        9,
        max(UPRIGHT_Y0 - 4.0, BOARD_Y1 + 3.0 + SWEEP_MARGIN + 0.5),
        UPRIGHT_Y0,
        zf0,
        zf1 + 6.0,
    )
    y = spigot + land + flange + arm_p + arm_n + upright + gusset
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
    # pan: centre-screw access through the axis, cross-horn pilots through
    y = y - _cyl_z(3.0, zl(Z_HORN_TOP) - 1, zf1 + 7, x=px)
    for k in range(4):
        a = math.radians(90 * k)
        y = y - _cyl_z(
            PILOT / 2,
            zl(Z_HORN_TOP) - 1,
            zf1 + 7,
            x=px + CROSS_SCREW_R * math.cos(a),
            y=CROSS_SCREW_R * math.sin(a),
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


def _outrigger_foot_y() -> float:
    return OUTRIGGER_PLATE_Y0 - 3.0


OUTRIGGER_FOOT_Y = _outrigger_foot_y()


def build_outrigger() -> Part:
    zf1 = W(Z_FLANGE_TOP)
    plate = _box(-7, 7, OUTRIGGER_PLATE_Y0, OUTRIGGER_PLATE_Y1, zf1, 0) + _cyl_y(
        7.0, OUTRIGGER_PLATE_Y0, OUTRIGGER_PLATE_Y1
    )
    boss = _cyl_y(6.0, OUTRIGGER_PLATE_Y1 - 0.01, OUTRIGGER_BOSS_Y)
    pin = _cyl_y(TB_ID / 2 - 0.05, OUTRIGGER_BOSS_Y - 0.01, IDLER_FACE_Y + TB_W - 0.1)
    foot = _box(-9.5, 9.5, OUTRIGGER_FOOT_Y - 3.3, OUTRIGGER_PLATE_Y1, zf1, zf1 + 3.0)
    o = plate + boss + pin + foot
    for x in OUTRIGGER_SCREW_X:
        o = o - _cyl_z(1.1, zf1 - 1, zf1 + 4, x=x, y=OUTRIGGER_FOOT_Y)
    return o


def build_tower() -> Part:
    """Stationary: chassis flange, servo box (tabs screw up into the ledge),
    horn chamber, 6705ZZ seat. Built in world coordinates about the pan axis."""
    s = SERVO
    cx = s.shaft_from_end - s.length / 2  # case centre, servo x (= world x)
    half_tab = s.tab_span / 2
    tab_x0, tab_x1 = cx - half_tab - FIT, cx + half_tab + FIT
    box_x0, box_x1 = tab_x0 - WALL, tab_x1 + WALL
    hw = s.width / 2 + FIT
    tab_top = ZS - (s.height - s.tab_underside) + s.tab_thickness
    body = _box(box_x0, box_x1, -hw - WALL, hw + WALL, 0, Z_CHAMBER)
    cyl = _cyl_z(TOWER_R, tab_top, Z_TOWER_TOP)
    flange = _box(-25.0, 19.0, -20.0, 20.0, 0, CHASSIS_FLANGE_T)
    t = body + cyl + flange
    # tab-span cavity open at the bottom, case cavity above the ledge
    t = t - _box(tab_x0, tab_x1, -hw, hw, -1, tab_top)
    t = t - _box(
        s.shaft_from_end - s.length - FIT_END,
        s.shaft_from_end + FIT_END,
        -hw,
        hw,
        tab_top - 1,
        Z_CHAMBER + 0.01,
    )
    t = t - _cyl_z(CHAMBER_R, Z_CHAMBER, Z_LIP0)
    t = t - _cyl_z(PB_ID / 2 + 1.0, Z_LIP0 - 0.01, Z_SEAT0 + 0.01)
    t = t - _cyl_z(PB_OD / 2, Z_SEAT0, Z_TOWER_TOP + 1)
    # lead exit slot at the -X end, open to the bottom
    boot_x0 = s.shaft_from_end - s.length - 6.0 - 1.0
    t = t - _box(boot_x0 - 5, tab_x0 + 0.5, -3.0, 3.0, -1, ZS - s.height + 10.0)
    # tab pilots, from below
    for x, _, z in fx.servo_tab_holes(s):
        t = t - _cyl_z(
            PILOT / 2, tab_top - 0.01, min(tab_top + 6.5, Z_CHAMBER + 0.5), x=x
        )
    for x, y in M3_HOLES:
        t = t - _cyl_z(1.7, -1, CHASSIS_FLANGE_T + 1, x=x, y=y)
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
    parts = [HEAD_LOC * fx.head(screws=True)]
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
    h = Pos(0, 0, ZS) * bored_horn("cross")
    screws = []
    for k in range(4):
        a = math.radians(90 * k)
        x, y = CROSS_SCREW_R * math.cos(a), CROSS_SCREW_R * math.sin(a)
        screws.append(_cyl_z(SCREW_CORE / 2, Z_HORN_TOP, Z_HORN_TOP + 5.0, x=x, y=y))
        screws.append(_cyl_z(1.6, Z_HORN_TOP - 1.6 - 1.2, Z_HORN_TOP - 1.6, x=x, y=y))
    return [h] + screws


def pan_bearing_world() -> Part:
    return _cyl_z(PB_OD / 2, Z_SEAT0, Z_TOWER_TOP) - _cyl_z(
        PB_ID / 2, Z_SEAT0 - 1, Z_TOWER_TOP + 1
    )


def chassis_screws_world() -> list[Part]:
    out = []
    for x, y in M3_HOLES:  # M3 x 10 socket head, nut under the acrylic
        out.append(_cyl_z(2.75, CHASSIS_FLANGE_T, CHASSIS_FLANGE_T + 3.0, x=x, y=y))
        out.append(_cyl_z(1.45, -6.0, CHASSIS_FLANGE_T, x=x, y=y))
    return out


# --------------------------------------------------------------------------
# Assembly
# --------------------------------------------------------------------------
def to_world() -> Location:
    return Pos(PAN_AXIS_BEHIND_LENS, 0, lens_height())


@functools.lru_cache(maxsize=1)
def _built() -> dict:
    return {
        "cradle": build_cradle(),
        "yoke": build_yoke(),
        "outrigger": build_outrigger(),
        "tower": build_tower(),
    }


def printed_parts() -> dict[str, Part]:
    """Every printed part in its WORLD pose at pan 0, tilt 0."""
    b = _built()
    w = to_world()
    return {
        "tower": b["tower"],
        "yoke": w * b["yoke"],
        "outrigger": w * b["outrigger"],
        "cradle": w * b["cradle"],
    }


COLOURS = {
    "tower": "#3d3d3d",
    "yoke": "#b54a68",
    "outrigger": "#d9822b",
    "cradle": "#3a7ca5",
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
    ]
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
        Body(f"tilt_servo{i}" if i else "tilt_servo", P(s), "tilt_servo")
        for i, s in enumerate(tilt_servo_bodies())
    ]
    out += [
        Body(
            "outrigger",
            P(b["outrigger"]),
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
    out += [
        Body(f"head{i}" if i else "head", T(s), "head")
        for i, s in enumerate(head_bodies())
    ]
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


def main() -> None:
    out = HERE / "output"
    out.mkdir(exist_ok=True)
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
    if abs(c.X + PAN_AXIS_BEHIND_LENS) > 0.5:
        print("WARNING: set PAN_AXIS_BEHIND_LENS to", round(-c.X, 1))


if __name__ == "__main__":
    main()
