"""Where a socketed sensor module's axes point on the robot, from its layout.

A module such as the GY-521 is described once in its footprint's frame: its
pads, its outline, and its sensor axes (`axes` in
`lib/breakout-sockets/sockets.toml`). The carrier fits that footprint with one
rigid placement (board.toml `[placement]`), and the module sits in its socket
component side up or down (board.toml `[imu] face`). This module turns those
into the 3x3 matrix R that takes a vector in the sensor's axes to the robot's
body frame (X forward, Z up, Y = Z x X, to the left), so `kicad.boardcheck`
can compare it with the firmware mount the board declares (`[imu] mount`).

Frames. Footprint and board coordinates are KiCad's: x right, y down, seen
from the board's top face, so their right-handed z points into the board.
Everything here is computed in the frame U seen from above: x right, y up
(board -y), z out of the top face. A footprint vector (x, y) is (x, -y, 0) in
U. KiCad's positive rotation turns a footprint counter-clockwise on screen,
which is the usual Rz(theta) in U (at 90 degrees local +x lands on board -y).

    R = M . Rz(theta) . F . A

- A: the module's axes in its footprint, as columns: X from `axes.x` (a unit
  vector along the footprint's x or y), Z out of the component face
  (`"component"`) or the solder face (`"solder"`), Y = Z x X.
- F: identity for `face = "up"`; for `"down"` the module turned over about
  the footprint's x axis, Rx(180) = diag(1, -1, -1). For a single pad row
  along x (the GY521_Socket) that is the one turn that puts every pin back
  on its own hole; boardcheck refuses `face = "down"` on any other footprint.
- Rz(theta): the placement rotation, a multiple of 90 degrees.
- M: board to robot, rows the robot's axes in U: X along `[outline]
  forward`, Z out of the board's top face (the carrier is mounted top face
  up), Y = Z x X.

R is always a signed permutation with determinant +1, one of the 24
axis-aligned rotations. The firmware supports two (`imu_mount_accel()` in
packages/robotics/balancebot/src/imu_filter.c): `up`, the identity, and
`down`, diag(-1, 1, -1), a 180 degree turn about body Y (the axle).

Stdlib and pure: no file is read here.
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import KicadError

Vector = tuple[int, int, int]
Matrix = tuple[Vector, Vector, Vector]

IDENTITY: Matrix = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
# The firmware's mounts (`mount up|down` on the robot's console).
MOUNTS: dict[str, Matrix] = {
    "up": IDENTITY,
    "down": ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),
}
FACES = ("up", "down")
# Board-frame (y down) unit vectors; `[outline] forward` takes these names.
DIRECTIONS: dict[str, tuple[int, int]] = {
    "+x": (1, 0),
    "-x": (-1, 0),
    "+y": (0, 1),
    "-y": (0, -1),
}
Z_FACES = {"component": 1, "solder": -1}
AXES_KEYS = frozenset({"x", "z"})
_RZ = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}  # (cos, sin)


@dataclass(frozen=True)
class ModuleAxes:
    """A module's sensor axes in its footprint's frame (x right, y down)."""

    x: tuple[int, int]  # the X axis: a unit vector along footprint x or y
    z: str  # the face the Z axis points out of: "component" or "solder"


def parse_axes(value: object, where: str) -> ModuleAxes:
    """A sockets.toml `axes = { x = [dx, dy], z = "component" }`, validated."""
    if not isinstance(value, dict) or set(value) != AXES_KEYS:
        raise KicadError(
            f'{where}: axes must be {{ x = [dx, dy], z = "component" | "solder" }}, '
            f"got {value!r}"
        )
    x = value["x"]
    if (
        not isinstance(x, list)
        or len(x) != 2
        or not all(isinstance(v, int) and not isinstance(v, bool) for v in x)
        or tuple(x) not in DIRECTIONS.values()
    ):
        raise KicadError(
            f"{where}: axes.x must be a unit vector along the footprint's x or y "
            f"([1, 0], [-1, 0], [0, 1] or [0, -1]; y is down), got {x!r}"
        )
    if value["z"] not in Z_FACES:
        raise KicadError(
            f"{where}: axes.z must be one of {sorted(Z_FACES)}, got {value['z']!r}"
        )
    return ModuleAxes((x[0], x[1]), value["z"])


def parse_forward(value: object, where: str) -> tuple[int, int]:
    """`[outline] forward`, one of DIRECTIONS, as a board-frame unit vector."""
    if not isinstance(value, str) or value not in DIRECTIONS:
        raise KicadError(
            f"{where}: [outline] forward must be one of {list(DIRECTIONS)}, got {value!r}"
        )
    return DIRECTIONS[value]


def quarter_turns(rotation_deg: float, where: str) -> int:
    """`rotation_deg` normalised to 0, 90, 180 or 270; anything else raises."""
    turns = rotation_deg / 90
    if turns != int(turns):
        raise KicadError(
            f"{where}: rotation {rotation_deg} is not a multiple of 90 degrees, so "
            "the module's axes are not along the robot's"
        )
    return int(turns) % 4 * 90


def _u(v: tuple[int, int]) -> Vector:
    """A board/footprint vector (y down) in U (y up)."""
    return (v[0], -v[1], 0)


def _cross(a: Vector, b: Vector) -> Vector:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _apply(m: Matrix, v: Vector) -> Vector:
    return tuple(sum(m[r][c] * v[c] for c in range(3)) for r in range(3))  # type: ignore[return-value]


def _columns(cols: tuple[Vector, Vector, Vector]) -> Matrix:
    return tuple(tuple(cols[c][r] for c in range(3)) for r in range(3))  # type: ignore[return-value]


def rotation(
    rotation_deg: float, face: str, axes: ModuleAxes, forward: tuple[int, int]
) -> Matrix:
    """R: sensor axes to robot body axes, for one placement (module docstring)."""
    if face not in FACES:
        raise KicadError(f"face must be one of {list(FACES)}, got {face!r}")
    if forward not in DIRECTIONS.values():
        raise KicadError(f"forward must be a board unit vector, got {forward!r}")
    cos, sin = _RZ[quarter_turns(rotation_deg, "placement")]
    rz: Matrix = ((cos, -sin, 0), (sin, cos, 0), (0, 0, 1))
    flip: Matrix = MOUNTS["up"] if face == "up" else ((1, 0, 0), (0, -1, 0), (0, 0, -1))
    x_r = _u(forward)
    z_r: Vector = (0, 0, 1)
    to_robot: Matrix = (x_r, _cross(z_r, x_r), z_r)
    x_m = _u(axes.x)
    z_m: Vector = (0, 0, Z_FACES[axes.z])
    sensor = (x_m, _cross(z_m, x_m), z_m)
    return _columns(
        tuple(_apply(to_robot, _apply(rz, _apply(flip, v))) for v in sensor)  # type: ignore[arg-type]
    )


def mount_for(matrix: Matrix) -> str | None:
    """The firmware mount whose matrix is `matrix`, or None if neither is."""
    for name, m in MOUNTS.items():
        if m == matrix:
            return name
    return None


def fmt(matrix: Matrix) -> str:
    """`matrix` on one line, rows in brackets."""
    return (
        "["
        + ", ".join("[" + ", ".join(f"{v:d}" for v in row) + "]" for row in matrix)
        + "]"
    )
