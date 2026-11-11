"""cadstd.shapes and cadstd.coupon (build123d).

Skipped where build123d is not installed (the stdlib `cadstd` pre-commit
hook); `just cadstd::test` runs them in tools/mechanism's venv and sets
CADSTD_REQUIRE_BUILD123D=1, which turns the skip into a failure.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import unittest
from pathlib import Path

try:
    from build123d import Vector

    from cadstd import coupon, fits, grid, shapes
    from cadstd.fasteners import M2, M3
    from cadstd.fits import StandardError

    HAVE_B3D = True
except ModuleNotFoundError:
    if os.environ.get("CADSTD_REQUIRE_BUILD123D"):
        raise
    HAVE_B3D = False


def _thinnest():
    """tools/enclosure's wall gate, loaded by path as tools/mechanism does."""
    path = Path(__file__).resolve().parents[1] / "enclosure/enclosure/thickness.py"
    spec = importlib.util.spec_from_file_location("enclosure_thickness", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod.thinnest


@unittest.skipUnless(HAVE_B3D, "build123d not installed")
class Shapes(unittest.TestCase):
    def test_bore_stands_on_xy_and_runs_up(self):
        bb = shapes.clearance_hole(M3, 5.0).bounding_box()
        self.assertAlmostEqual(bb.min.Z, 0.0)
        self.assertAlmostEqual(bb.max.Z, 5.0)
        self.assertAlmostEqual(bb.size.X, M3.clearance, places=6)

    def test_nut_pocket_is_the_nut_plus_a_close_fit_across_flats(self):
        bb = shapes.nut_pocket(M3, 2.4, "mjf").bounding_box()
        self.assertAlmostEqual(bb.size.Y, 5.8, places=6)  # flats parallel to X
        self.assertAlmostEqual(bb.size.X, 5.8 * 2 / 3**0.5, places=6)

    def test_insert_hole_refuses_resin(self):
        with self.assertRaises(StandardError):
            shapes.insert_hole(M3, "sla", 5.0)
        self.assertAlmostEqual(
            shapes.insert_hole(M2, "mjf", 3.0).bounding_box().size.X, 3.2, places=6
        )

    def test_slot_travels_the_float_each_way(self):
        bb = shapes.slot(M3, 3.0).bounding_box()
        self.assertAlmostEqual(bb.size.X, grid.slot_length(M3), places=6)
        self.assertAlmostEqual(bb.size.Y, M3.clearance, places=6)

    def test_grid_plate_has_a_hole_at_every_grid_point(self):
        nx, ny, t = 3, 2, grid.PLATES[0]
        p = shapes.grid_plate(nx, ny, t)
        self.assertTrue(p.is_valid)
        self.assertAlmostEqual(p.bounding_box().size.X, grid.plate_size(nx), places=6)
        for x, y in grid.points(nx, ny):
            self.assertFalse(p.is_inside(Vector(x, y, t / 2)), (x, y))
            self.assertTrue(p.is_inside(Vector(x + grid.PITCH / 2 - 1, y, t / 2)))


@unittest.skipUnless(HAVE_B3D, "build123d not installed")
class Coupon(unittest.TestCase):
    def test_every_process_builds_one_valid_solid_per_part(self):
        for process in fits.PROCESSES:
            for name, part in coupon.coupon(process).items():
                self.assertTrue(part.is_valid, (process, name))
                self.assertEqual(len(part.solids()), 1, (process, name))

    def test_insert_holes_only_on_processes_that_take_them(self):
        """Inserts add two through holes; resin's plate keeps that material."""
        sla, mjf = coupon.plate("sla"), coupon.plate("mjf")
        self.assertGreater(sla.volume, mjf.volume)

    def test_ladder_holes_grow_from_the_chamfered_corner(self):
        p = coupon.plate("fdm")
        for (x, _), gap in zip(
            grid.points(len(coupon.LADDER), 1), coupon.LADDER, strict=True
        ):
            r = coupon.PEG_D / 2 + gap
            self.assertFalse(p.is_inside(Vector(x + r - 0.01, 0, 1)), gap)
            self.assertTrue(p.is_inside(Vector(x + r + 0.01, 0, 1)), gap)

    def test_coupon_passes_the_wall_gate(self):
        """It is ordered like any part, so JLC3DP must not flag it."""
        thinnest = _thinnest()
        for process in ("fdm", "mjf"):  # the two back-row variants
            t = thinnest(coupon.plate(process))
            self.assertGreaterEqual(t.mm, fits.MIN_WALL - 0.05, (process, t.where))


if __name__ == "__main__":
    unittest.main()
