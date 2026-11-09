"""Tests for `kicad.mount`: a socketed module's sensor axes on the robot.

The expected matrices come from the physical cases, worked by hand in each
test's comment: where the module's X arrow and Z face point on the robot. The
sweep compares `mount.rotation` with a second derivation that never leaves
KiCad's y-down board frame: the arrow is turned with `perfboard.grid.rotate`
(KiCad's footprint rotation, checked there) and read off against the board's
forward and left directions.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import itertools
import unittest

from kicad import mount, sockets
from kicad.errors import KicadError
from kicad.perfboard.grid import rotate

# The GY-521's axes as the board page draws them: X arrow away from the header
# (footprint -y, the module lies toward -y), Z out of the component face.
GY521 = mount.ModuleAxes((0, -1), "component")
FORWARD = mount.DIRECTIONS["-y"]
UP = mount.MOUNTS["up"]
DOWN = mount.MOUNTS["down"]


def det(m) -> int:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def signed_permutation(m) -> bool:
    rows_ok = all(sorted(abs(v) for v in row) == [0, 0, 1] for row in m)
    cols_ok = all(sorted(abs(m[r][c]) for r in range(3)) == [0, 0, 1] for c in range(3))
    return rows_ok and cols_ok


def by_hand(rot, face, axes, forward):
    """R from the board frame alone: turn the arrow, read it on the robot.

    Flipping the module about the pad row (footprint x) reverses the arrow's y
    and turns Z over; KiCad then rotates the footprint. On a board seen from
    above with y down, the robot's left is forward turned a quarter
    counter-clockwise on screen: rotate(forward, 90).
    """
    ax, ay = axes.x
    z = mount.Z_FACES[axes.z]
    if face == "down":
        ay, z = -ay, -z
    x, y = rotate(ax, ay, rot)
    fx, fy = forward
    lx, ly = rotate(fx, fy, 90)
    xcol = (x * fx + y * fy, x * lx + y * ly, 0)
    zcol = (0, 0, z)
    ycol = (  # Z x X
        zcol[1] * xcol[2] - zcol[2] * xcol[1],
        zcol[2] * xcol[0] - zcol[0] * xcol[2],
        zcol[0] * xcol[1] - zcol[1] * xcol[0],
    )
    cols = (xcol, ycol, zcol)
    return tuple(tuple(cols[c][r] for c in range(3)) for r in range(3))


class PhysicalCaseTests(unittest.TestCase):
    def test_rev3_layout_is_mount_up(self):
        # J3 at rotation 0 on #1015's board: the module lies toward board -y,
        # which is forward, component side up. X arrow forward, Z up.
        self.assertEqual(mount.rotation(0, "up", GY521, FORWARD), UP)

    def test_module_face_down_in_the_rev3_socket_is_mount_down(self):
        # The 8d18cd8d case: turned over about the pin row, the module hangs
        # rearward. X arrow back, Z down, Y (along the row) unchanged:
        # 180 degrees about the axle, the firmware's `mount down`.
        self.assertEqual(mount.rotation(0, "down", GY521, FORWARD), DOWN)

    def test_socket_turned_180_points_x_back_with_z_up(self):
        # The footprint turned half round: arrow back, component side still
        # up, so Y (the row) reverses too. 180 degrees about Z; no firmware
        # mount reads it.
        r = mount.rotation(180, "up", GY521, FORWARD)
        self.assertEqual(r, ((-1, 0, 0), (0, -1, 0), (0, 0, 1)))
        self.assertIsNone(mount.mount_for(r))

    def test_kicad_positive_rotation_turns_the_arrow_to_the_left(self):
        # KiCad's +90 is counter-clockwise on screen: the arrow that pointed
        # forward (up the page) now points left, the robot's +Y. Sensor X
        # reads along body Y, sensor Y along body -X.
        r = mount.rotation(90, "up", GY521, FORWARD)
        self.assertEqual(r, ((0, -1, 0), (1, 0, 0), (0, 0, 1)))

    def test_forward_backward_is_the_half_turn(self):
        # Same J3, but the robot drives toward board +y: the arrow points back.
        r = mount.rotation(0, "up", GY521, mount.DIRECTIONS["+y"])
        self.assertEqual(r, ((-1, 0, 0), (0, -1, 0), (0, 0, 1)))

    def test_forward_plus_x_puts_the_arrow_on_the_left(self):
        # Robot forward is board +x (right); the arrow points up the page,
        # which is the robot's left.
        r = mount.rotation(0, "up", GY521, mount.DIRECTIONS["+x"])
        self.assertEqual(r, ((0, -1, 0), (1, 0, 0), (0, 0, 1)))

    def test_turning_board_forward_and_socket_together_changes_nothing(self):
        # Forward turned a quarter counter-clockwise (-y to -x) with the
        # socket turned the same way: the module still faces forward.
        for fwd, rot in (("-y", 0), ("-x", 90), ("+y", 180), ("+x", 270)):
            with self.subTest(forward=fwd):
                r = mount.rotation(rot, "up", GY521, mount.DIRECTIONS[fwd])
                self.assertEqual(r, UP)

    def test_solder_side_z_is_the_face_down_module_with_its_arrow_reversed(self):
        # Declaring Z out of the solder face is the same physical sensor as
        # a component-face Z with the module turned over about the row.
        solder = mount.ModuleAxes((0, 1), "solder")
        self.assertEqual(mount.rotation(0, "up", solder, FORWARD), DOWN)

    def test_every_quarter_turn_and_face_is_a_proper_signed_permutation(self):
        for rot, face, fwd in itertools.product(
            (0, 90, 180, 270), mount.FACES, mount.DIRECTIONS
        ):
            with self.subTest(rot=rot, face=face, forward=fwd):
                r = mount.rotation(rot, face, GY521, mount.DIRECTIONS[fwd])
                self.assertTrue(signed_permutation(r), r)
                self.assertEqual(det(r), 1, r)

    def test_matches_the_board_frame_derivation(self):
        for rot, face, fwd, x, z in itertools.product(
            (0, 90, 180, 270, -90, 450),
            mount.FACES,
            mount.DIRECTIONS.values(),
            mount.DIRECTIONS.values(),
            mount.Z_FACES,
        ):
            axes = mount.ModuleAxes(x, z)
            with self.subTest(rot=rot, face=face, forward=fwd, axes=axes):
                self.assertEqual(
                    mount.rotation(rot, face, axes, fwd),
                    by_hand(rot % 360, face, axes, fwd),
                )

    def test_firmware_mounts_are_named(self):
        self.assertEqual(mount.mount_for(UP), "up")
        self.assertEqual(mount.mount_for(DOWN), "down")
        self.assertEqual(det(DOWN), 1)


class DeclaredAxesTests(unittest.TestCase):
    def test_gy521_socket_axes_at_the_rev3_placement_give_mount_up(self):
        # sockets.toml's declared axes, not this file's copy, with #1015's
        # placement (J3 at rotation 0, forward -y). The board itself is
        # checked by boardcheck in CI; this pins the library entry.
        (socket,) = [
            s
            for s in sockets.load_config(sockets.subtree() / sockets.CONFIG)
            if s.name == "GY521_Socket"
        ]
        self.assertEqual(mount.rotation(0, "up", socket.axes, FORWARD), UP)
        self.assertEqual(mount.rotation(0, "down", socket.axes, FORWARD), DOWN)


class ParseTests(unittest.TestCase):
    def test_axes_accepts_unit_vectors_and_faces(self):
        a = mount.parse_axes({"x": [0, -1], "z": "component"}, "t")
        self.assertEqual(a, GY521)
        self.assertEqual(
            mount.parse_axes({"x": [1, 0], "z": "solder"}, "t"),
            mount.ModuleAxes((1, 0), "solder"),
        )

    def test_axes_rejects(self):
        bad = [
            {"x": [0, -1]},
            {"x": [0, -1], "z": "component", "y": [1, 0]},
            {"x": [1, 1], "z": "component"},
            {"x": [0, -2], "z": "component"},
            {"x": [0.0, -1.0], "z": "component"},
            {"x": [True, 0], "z": "component"},
            {"x": [0, 0, 1], "z": "component"},
            {"x": [0, -1], "z": "up"},
            {"x": [0, -1], "z": ["component"]},
            {"x": [0, -1], "z": {"face": "component"}},
            {"x": "+x", "z": "component"},
            [0, -1],
        ]
        for value in bad:
            with self.subTest(value=value), self.assertRaisesRegex(KicadError, "axes"):
                mount.parse_axes(value, "t")

    def test_forward(self):
        for name, v in mount.DIRECTIONS.items():
            self.assertEqual(mount.parse_forward(name, "t"), v)
        for bad in ("y", "-Y", "up", None, 1):
            with self.subTest(bad=bad), self.assertRaisesRegex(KicadError, "forward"):
                mount.parse_forward(bad, "t")

    def test_rotation_must_be_a_quarter_turn(self):
        with self.assertRaisesRegex(KicadError, "not a multiple of 90"):
            mount.rotation(45, "up", GY521, FORWARD)
        self.assertEqual(mount.rotation(360.0, "up", GY521, FORWARD), UP)

    def test_face_must_be_up_or_down(self):
        with self.assertRaisesRegex(KicadError, "face"):
            mount.rotation(0, "sideways", GY521, FORWARD)

    def test_fmt(self):
        self.assertEqual(mount.fmt(DOWN), "[[-1, 0, 0], [0, 1, 0], [0, 0, -1]]")


if __name__ == "__main__":
    unittest.main()
