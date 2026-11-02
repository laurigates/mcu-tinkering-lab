"""Phone microscope stand for electronics inspection: a phone on a tray over an M12 lens.

The phone lies screen up on a tray. A stock M12x0.5 board-camera lens sits
under the phone's main camera, mounted reversed (its sensor side toward the
board), and the phone, focused at infinity or near it, looks through it. The
board lies on the bench under the lens.

- Coarse focus: the tray's split collar slides on a 10 mm column (steel or
  aluminium rod, or M10 threaded rod) and locks with a thumb knob. The phone's
  own autofocus does the fine focus.
- Alignment to any phone: the phone sits against a left and a back lip. Under
  the tray, an X slide moves +-20 mm in X and a lens plate moves +-20 mm in Y
  on it, so the lens reaches a camera 11..51 mm from the phone's left edge
  and 11..51 mm from its top edge, case included. Each axis locks with two
  M3 button-head screws and printed thumb knobs.
- The lens is held by its thread in a split collar on the lens plate (no
  printed thread); it slides up until its front is just under the camera
  bump, and an M3 screw clamps it.

Frames: world +X right, +Y away from the user (toward the column), +Z up, mm;
the bench top is z 0. Parts are built in the STAGE frame: origin at the lens
centre at the nominal carriage pose (x 0, y 0), on the tray's top face.
``assembly(x, y, height)`` puts the tray top at world z = height.

Every dimension the fit depends on is a parameter below; the ones marked
"estimate" are in README.md under "Measure before printing".
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import (
    Align,
    Box,
    Cylinder,
    Location,
    Part,
    Pos,
    Rot,
    export_step,
    export_stl,
)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "tools/mechanism"))
from mechanism.check import Body  # noqa: E402

# --- Off-the-shelf parts -----------------------------------------------------
ROD_D = 10.0  # column: 10 mm rod or M10 threaded rod
ROD_L = 250.0
ROD_FIT = 0.3  # bore clearance on the rod; the split collars close it up

LENS_THREAD_D = 12.0  # M12x0.5 major diameter
LENS_FIT = 0.1  # collar bore 12.1; the clamp screw closes the slit
LENS_FRONT_D = 16.0  # estimate: widest part of the lens body above the thread
LENS_FRONT_H = 6.0  # estimate: height of that wider front section
LENS_L = 21.0  # estimate: overall lens length (8..25 mm M12 lenses: ~15..25)
CAMERA_BUMP = 2.0  # estimate: camera bump proud of the phone back (case: less)
LENS_BELOW_BUMP = 0.5  # gap left between the lens front and the camera glass

# Phone stand-in: a large phone in a case (160 x 78 x 9); the camera sits over
# the lens at the nominal pose, 31 mm from the left and top edges.
PHONE = (160.0, 78.0, 9.0)

# M3 button head (ISO 7380): head 5.7 x 1.65; M3 nut 5.5 AF x 2.4.
M3_CLEAR = 3.4
M3_HEAD = (5.7, 1.65)
M3_NUT_AF = 5.5
M3_NUT_T = 2.4
M4_CLEAR = 4.4
M4_HEAD = (7.0, 4.0)  # socket head
M4_NUT_AF = 7.0
M4_NUT_T = 3.2

# --- Layout ------------------------------------------------------------------
TRAVEL = 20.0  # lens carriage travel, +- in X and in Y
LIP_IN = 31.0  # left lip inner face at x -31, back lip inner face at y +31
LIP_T = 4.0
LIP_H = 6.0
TRAY_T = 6.0
TRAY_X = (-LIP_IN - LIP_T, 130.0)
TRAY_Y = (-62.0, 66.0)
WINDOW = 30.0  # tray window +-30 in X and Y

TRACK_W = 6.2  # counterbore track for the button heads
TRACK_D = 2.0
SLOT_Y = 44.0  # tray slots for the X screws, at y +-44 (the back one behind the lip)

XS_T = 4.0  # X slide plate
XS_X = 26.0  # +- extent
XS_Y = 48.0
XS_WIN_X = LENS_FRONT_D / 2 + 1.5  # lens front clearance in the X slide
YSLOT_X = 18.0  # X slide slots for the Y screws, at x +-18

LP_T = 4.0  # lens plate
LP_X = 24.0
LP_Y = 11.0
COLLAR_OD = LENS_THREAD_D + LENS_FIT + 6.0
COLLAR_H = 10.0  # below the lens plate
SLIT = 1.5
EAR_T = 4.0
EAR_TO = 16.0  # clamp ears reach y -16 from the lens axis (2.3 mm past the bolt hole)
EAR_BOLT_Y = -12.0

KNOB_D = 14.0  # printed thumb knobs on M3 nuts
KNOB_H = 6.0
KNOB4_D = 16.0
KNOB4_H = 8.0

COLLAR_Y = 72.0  # column axis in the stage frame (x 0); clears the back X knob at the lowest height
ROD_COLLAR_OD = ROD_D + ROD_FIT + 10.0
ROD_COLLAR_Z = (-TRAY_T, 24.0)
BACK_WALL = (50.0, 64.0)  # y span of the stiffening wall behind the back track
BACK_WALL_H = 12.0
ROD_EAR = (6.0, 22.0)  # clamp ears: y from collar wall, length
ROD_SLIT = 2.0

BASE_T = 8.0
BASE_X = (-70.0, 140.0)
BASE_LEG_W = 20.0
BASE_FRONT = -90.0
SOCKET_OD = 24.0
SOCKET_H = 20.0
SOCKET_FLOOR = 3.0

# Stage-frame z of each layer
Z_XS = -TRAY_T  # X slide top
Z_LP = Z_XS - XS_T  # lens plate top
Z_LP_BOT = Z_LP - LP_T
LENS_TOP = -CAMERA_BUMP - LENS_BELOW_BUMP

HEIGHT_MIN = SOCKET_H + TRAY_T + 1.0  # collar bottom 1 mm above the socket

JOINTS = ("x", "y", "height")
HEIGHTS = (HEIGHT_MIN, 120.0, ROD_L + SOCKET_FLOOR - ROD_COLLAR_Z[1])
SWEEP = [
    (x, y, h)
    for x in (-TRAVEL, 0, TRAVEL)
    for y in (-TRAVEL, 0, TRAVEL)
    for h in HEIGHTS
]
REST = (0.0, 0.0, 60.0)
SHOWCASE_POSES = [("corner-x20-y-20", (TRAVEL, -TRAVEL, 60.0))]


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Part:
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN,) * 3)


def cyl(d: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> Part:
    return Pos(x, y, z0) * Cylinder(
        d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def xcyl(d: float, x0: float, x1: float, y: float, z: float) -> Part:
    """A cylinder along X."""
    return Pos(x0, y, z) * (
        Rot(0, 90, 0)
        * Cylinder(d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def slot(
    length_x: float,
    width: float,
    z0: float,
    z1: float,
    x: float = 0.0,
    y: float = 0.0,
    along_y: bool = False,
) -> Part:
    """A stadium slot whose centre line runs +-length/2 about (x, y)."""
    h = length_x / 2
    if along_y:
        core = box(x - width / 2, x + width / 2, y - h, y + h, z0, z1)
        return core + cyl(width, z0, z1, x, y - h) + cyl(width, z0, z1, x, y + h)
    core = box(x - h, x + h, y - width / 2, y + width / 2, z0, z1)
    return core + cyl(width, z0, z1, x - h, y) + cyl(width, z0, z1, x + h, y)


def hexagon(af: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> Part:
    """A hex prism, across-flats af, flats parallel to X."""
    from build123d import RegularPolygon, extrude

    return Pos(x, y, z0) * extrude(
        RegularPolygon(af / 2, 6, major_radius=False), z1 - z0
    )


def split_clamp(
    bore: float,
    od: float,
    z0: float,
    z1: float,
    cx: float,
    cy: float,
    slit: float,
    ear_t: float,
    ear_len: float,
) -> tuple[Part, Part]:
    """A collar with a slit toward +Y and two clamp ears; returns (solid, cut)."""
    r = od / 2
    solid = cyl(od, z0, z1, cx, cy) + box(
        cx - slit / 2 - ear_t, cx + slit / 2 + ear_t, cy, cy + r + ear_len, z0, z1
    )
    cut = cyl(bore, z0 - 1, z1 + 1, cx, cy) + box(
        cx - slit / 2, cx + slit / 2, cy, cy + r + ear_len + 1, z0 - 1, z1 + 1
    )
    return solid, cut


# --- Printed parts -------------------------------------------------------------
def tray() -> Part:
    """Phone tray, lips, back wall and the column collar, in one print (bottom down)."""
    x0, x1 = TRAY_X
    y0, y1 = TRAY_Y
    p = box(x0, x1, y0, y1, -TRAY_T, 0)
    p += box(x0, -LIP_IN, y0, LIP_IN + LIP_T, 0, LIP_H)  # left lip
    p += box(x0, x1, LIP_IN, LIP_IN + LIP_T, 0, LIP_H)  # back lip
    p += box(x0, 60.0, *BACK_WALL, 0, BACK_WALL_H)  # back wall, behind the back track
    ear_len = ROD_EAR[0] + ROD_EAR[1] - ROD_COLLAR_OD / 2
    c_solid, c_cut = split_clamp(
        ROD_D + ROD_FIT,
        ROD_COLLAR_OD,
        *ROD_COLLAR_Z,
        0,
        COLLAR_Y,
        ROD_SLIT,
        5.0,
        ear_len,
    )
    p += c_solid
    p -= c_cut
    p -= rod_clamp_bolt_hole(COLLAR_Y, sum(ROD_COLLAR_Z) / 2)
    p -= box(-WINDOW, WINDOW, -WINDOW, WINDOW, -TRAY_T - 1, 1)
    for y in (-SLOT_Y, SLOT_Y):
        p -= slot(2 * TRAVEL, M3_CLEAR, -TRAY_T - 1, 1, 0, y)
        p -= slot(2 * TRAVEL, TRACK_W, -TRACK_D, 1, 0, y)
    return p


def rod_clamp_bolt_y(cy: float) -> float:
    return (
        cy + ROD_COLLAR_OD / 2 + ROD_EAR[0] + ROD_EAR[1] / 2 - ROD_COLLAR_OD / 2 + 4.0
    )


def rod_clamp_bolt_hole(cy: float, z: float) -> Part:
    return xcyl(M4_CLEAR, -20, 20, rod_clamp_bolt_y(cy), z)


def x_slide() -> Part:
    p = box(-XS_X, XS_X, -XS_Y, XS_Y, Z_LP, Z_XS)
    p -= slot(2 * TRAVEL, 2 * XS_WIN_X, Z_LP - 1, Z_XS + 1, 0, 0, along_y=True)
    for x in (-YSLOT_X, YSLOT_X):
        p -= slot(2 * TRAVEL, M3_CLEAR, Z_LP - 1, Z_XS + 1, x, 0, along_y=True)
        p -= slot(2 * TRAVEL, TRACK_W, Z_XS - TRACK_D, Z_XS + 1, x, 0, along_y=True)
    for y in (-SLOT_Y, SLOT_Y):
        p -= cyl(M3_CLEAR, Z_LP - 1, Z_XS + 1, 0, y)
    return p


def lens_plate() -> Part:
    p = box(-LP_X, LP_X, -LP_Y, LP_Y, Z_LP_BOT, Z_LP)
    zc0 = Z_LP_BOT - COLLAR_H
    # collar and ears (ears toward -Y: build with the +Y helper, then mirror by rotation)
    solid, cut = split_clamp(
        LENS_THREAD_D + LENS_FIT,
        COLLAR_OD,
        zc0,
        Z_LP_BOT,
        0,
        0,
        SLIT,
        EAR_T,
        EAR_TO - COLLAR_OD / 2,
    )
    flip = Rot(0, 0, 180)
    p += flip * solid
    p -= flip * cut
    p -= cyl(
        LENS_FRONT_D + 1.0, Z_LP_BOT - 0.01, Z_LP + 1
    )  # lens front passes the plate
    p -= cyl(LENS_THREAD_D + LENS_FIT, zc0 - 1, Z_LP + 1)
    p -= xcyl(M3_CLEAR, -15, 15, EAR_BOLT_Y, (zc0 + Z_LP_BOT) / 2)
    for x in (-YSLOT_X, YSLOT_X):
        p -= cyl(M3_CLEAR, Z_LP_BOT - 1, Z_LP + 1, x, 0)
    return p


def knob(d: float, h: float, clear: float, nut_af: float, nut_t: float) -> Part:
    """Thumb knob, nut pocket at z 0 (the face away from the plate), print pocket up."""
    from build123d import RegularPolygon, extrude

    p = Pos(0, 0, 0) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    n = 12
    import math

    for i in range(n):
        a = 2 * math.pi * i / n
        p -= cyl(
            2.4, -1, h + 1, (d / 2 + 0.6) * math.cos(a), (d / 2 + 0.6) * math.sin(a)
        )
    p -= cyl(clear, -1, h + 1)
    p -= extrude(RegularPolygon((nut_af + 0.3) / 2, 6, major_radius=False), nut_t + 0.3)
    return p


def base() -> Part:
    x0, x1 = BASE_X
    cy = COLLAR_Y
    p = box(x0, x1, cy - 14, cy + 12, 0, BASE_T)  # back bar
    p += box(x0, x0 + BASE_LEG_W, BASE_FRONT, cy + 12, 0, BASE_T)
    p += box(x1 - BASE_LEG_W, x1, BASE_FRONT, cy + 12, 0, BASE_T)
    ear_len = ROD_EAR[0] + ROD_EAR[1] - SOCKET_OD / 2
    s_solid, s_cut = split_clamp(
        ROD_D + ROD_FIT, SOCKET_OD, 0, SOCKET_H, 0, cy, ROD_SLIT, 5.0, ear_len
    )
    p += s_solid
    p -= s_cut - cyl(SOCKET_OD + 2, -2, SOCKET_FLOOR)  # floor under the rod
    p -= box(-ROD_SLIT / 2, ROD_SLIT / 2, cy, cy + 60, SOCKET_FLOOR, SOCKET_H + 1)
    p -= xcyl(
        M4_CLEAR,
        -20,
        20,
        rod_clamp_bolt_y(cy),
        SOCKET_FLOOR + (SOCKET_H - SOCKET_FLOOR) / 2,
    )
    for x in (x0 + BASE_LEG_W / 2, x1 - BASE_LEG_W / 2):  # rubber feet recesses
        for y in (BASE_FRONT + 12, cy):
            p -= cyl(12.5, -1, 1.0, x, y)
    return p


def printed_parts() -> dict[str, Part]:
    return {
        "tray": tray(),
        "x_slide": x_slide(),
        "lens_plate": lens_plate(),
        "base": base(),
        "knob_m3": knob(KNOB_D, KNOB_H, M3_CLEAR - 0.2, M3_NUT_AF, M3_NUT_T),
        "knob_m4": knob(KNOB4_D, KNOB4_H, M4_CLEAR - 0.2, M4_NUT_AF, M4_NUT_T),
    }


# --- Stand-ins -----------------------------------------------------------------
def phone() -> Part:
    w, d, t = PHONE
    p = box(-LIP_IN, -LIP_IN + w, LIP_IN - d, LIP_IN, 0, t)
    return p + cyl(14.0, -CAMERA_BUMP, 0)


def lens() -> Part:
    z_front0 = LENS_TOP - LENS_FRONT_H
    return cyl(LENS_FRONT_D, z_front0, LENS_TOP) + cyl(
        LENS_THREAD_D, LENS_TOP - LENS_L, z_front0
    )


def button_screw(x: float, y: float, z_head_bottom: float, length: float) -> Part:
    h, t = M3_HEAD[0], M3_HEAD[1]
    return cyl(h, z_head_bottom, z_head_bottom + t, x, y) + cyl(
        3.0, z_head_bottom - length, z_head_bottom, x, y
    )


def knob_at(
    x: float, y: float, z_top: float, d: float = KNOB_D, h: float = KNOB_H
) -> Part:
    return cyl(d, z_top - h, z_top, x, y)


def rod_clamp(cy: float, z: float) -> Part:
    """M4 x 35 socket head through the ears, knob on the +X side."""
    y = rod_clamp_bolt_y(cy)
    half = ROD_SLIT / 2 + 5.0
    return (
        xcyl(M4_HEAD[0], -half - M4_HEAD[1], -half, y, z)
        + xcyl(4.0, -half, -half + 35.0, y, z)
        + xcyl(KNOB4_D, half, half + KNOB4_H, y, z)
    )


def lens_clamp() -> Part:
    """M3 x 16 socket head across the lens collar's ears, nut on +X."""
    z = Z_LP_BOT - COLLAR_H / 2
    half = SLIT / 2 + EAR_T
    return (
        xcyl(5.5, -half - 3.0, -half, EAR_BOLT_Y, z)
        + xcyl(3.0, -half, -half + 16.0, EAR_BOLT_Y, z)
        + xcyl(6.4, half, half + M3_NUT_T, EAR_BOLT_Y, z)
    )


def assembly(x: float = 0.0, y: float = 0.0, height: float = REST[2]) -> list[Body]:
    stage = Location((0, -COLLAR_Y, 0))  # column axis at world x 0, y 0
    up = Pos(0, 0, height) * stage
    xs = up * Pos(x, 0, 0)
    lp = xs * Pos(0, y, 0)
    parts = printed_parts()
    x_screws = [button_screw(0, sy, -TRACK_D, 14.0) for sy in (-SLOT_Y, SLOT_Y)]
    x_knobs = [knob_at(0, sy, Z_LP) for sy in (-SLOT_Y, SLOT_Y)]
    y_screws = [button_screw(sx, 0, Z_XS - TRACK_D, 12.0) for sx in (-YSLOT_X, YSLOT_X)]
    y_knobs = [knob_at(sx, 0, Z_LP_BOT) for sx in (-YSLOT_X, YSLOT_X)]
    rod_z = sum(ROD_COLLAR_Z) / 2
    return [
        Body("base", stage * parts["base"], "base", printed=True, colour="#3a3a3a"),
        Body(
            "rod",
            stage * cyl(ROD_D, SOCKET_FLOOR, SOCKET_FLOOR + ROD_L, 0, COLLAR_Y),
            "rod",
        ),
        Body(
            "base_clamp",
            stage * rod_clamp(COLLAR_Y, SOCKET_FLOOR + (SOCKET_H - SOCKET_FLOOR) / 2),
            "base_clamp",
        ),
        Body("tray", up * parts["tray"], "tray", printed=True, colour="#d9822b"),
        Body("tray_clamp", up * rod_clamp(COLLAR_Y, rod_z), "tray_clamp"),
        Body("phone", up * phone(), "phone"),
        Body(
            "x_slide", xs * parts["x_slide"], "x_slide", printed=True, colour="#3a7ca5"
        ),
        *[Body(f"x_screw_{i}", xs * s, "x_screws") for i, s in enumerate(x_screws)],
        *[
            Body(f"x_knob_{i}", xs * k, "x_screws", printed=True, colour="#c9b037")
            for i, k in enumerate(x_knobs)
        ],
        Body(
            "lens_plate",
            lp * parts["lens_plate"],
            "lens_plate",
            printed=True,
            colour="#7a9e3d",
        ),
        *[Body(f"y_screw_{i}", lp * s, "y_screws") for i, s in enumerate(y_screws)],
        *[
            Body(f"y_knob_{i}", lp * k, "y_screws", printed=True, colour="#c9b037")
            for i, k in enumerate(y_knobs)
        ],
        Body("lens", lp * lens(), "lens"),
        Body("lens_clamp", lp * lens_clamp(), "lens_clamp"),
    ]


def working_distance_range() -> tuple[float, float]:
    """Lens rear face above the bench at the lowest and highest collar positions."""
    rear = LENS_TOP - LENS_L
    return HEIGHTS[0] + rear, HEIGHTS[-1] + rear


def main() -> None:
    for name, part in printed_parts().items():
        export_step(part, str(HERE / f"{name}.step"))
        export_stl(
            part, str(HERE / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1
        )
        bb = part.bounding_box().size
        print(
            f"{name}: {part.volume / 1000:.2f} cm3, bbox {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f}, valid {part.is_valid}"
        )
    lo, hi = working_distance_range()
    print(f"lens rear face above the bench: {lo:.1f} .. {hi:.1f} mm")


if __name__ == "__main__":
    main()
