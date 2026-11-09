"""cadstd: the fastener table, the fit profiles and the grid (ADR-030).

Stdlib only, so the `cadstd` pre-commit hook runs it without build123d. The
shape tests are in test_shapes.py.

    python3 -m unittest discover -s tools/cadstd -t tools
"""

from __future__ import annotations

import math
import unittest

from cadstd import fasteners, fits, grid
from cadstd.fasteners import M2, M3, SCREWS
from cadstd.fits import PROCESSES, StandardError


class Fasteners(unittest.TestCase):
    def test_sizes_are_ordered_and_named_by_diameter(self):
        ds = [s.d for s in SCREWS.values()]
        self.assertEqual(ds, sorted(ds))
        for name, s in SCREWS.items():
            self.assertEqual(name, s.name)
            self.assertEqual(name, f"M{s.d:g}")

    def test_pilot_under_nominal_under_clearance(self):
        for s in SCREWS.values():
            self.assertLess(s.pilot, s.d, s.name)
            self.assertGreater(s.clearance, s.d, s.name)

    def test_nut_fits_round_its_screw(self):
        for s in SCREWS.values():
            self.assertGreater(s.nut_af, s.clearance, s.name)

    def test_values_the_existing_parts_were_built_with(self):
        """case.py, the pan-tilt and the microscope used these before cadstd;
        a change here moves committed geometry (ADR-030, Migration)."""
        self.assertEqual((M3.clearance, M3.pilot), (3.4, 2.5))
        self.assertEqual(M2.pilot, 1.6)
        self.assertEqual((M3.nut_af, M3.nut_t), (5.5, 2.4))
        self.assertEqual(M3.button_head, (5.7, 1.65))
        self.assertEqual(SCREWS["M4"].socket_head, (7.0, 4.0))
        self.assertEqual((SCREWS["M4"].nut_af, SCREWS["M4"].nut_t), (7.0, 3.2))

    def test_lookup_fails_on_an_unknown_size(self):
        with self.assertRaises(StandardError):
            fasteners.screw("M5")
        self.assertIs(fasteners.screw("M3"), M3)


class Fits(unittest.TestCase):
    def test_every_process_defines_every_fit_in_order(self):
        for p in PROCESSES.values():
            self.assertEqual(set(p.fits), set(fits.FITS), p.name)
            gaps = [p.fits[k].gap for k in fits.FITS]
            self.assertEqual(gaps, sorted(gaps), p.name)

    def test_hole_adds_the_gap_on_both_sides(self):
        self.assertAlmostEqual(
            fits.hole(6.0, "slide", "fdm"), 6.0 + 2 * fits.fit("slide", "fdm")
        )

    def test_values_the_existing_parts_were_built_with(self):
        self.assertEqual(fits.fit("slide", "sla"), 0.3)  # case.py LIP_FIT
        self.assertEqual(fits.fit("slide", "mjf"), 0.3)  # pan-tilt KEY_FIT
        self.assertEqual(fits.fit("running", "mjf"), 0.5)  # pan-tilt GAP
        # pan-tilt M3 nut pocket 5.8 AF, microscope rod bore 10.3
        self.assertAlmostEqual(fits.hole(M3.nut_af, "close", "mjf"), 5.8)
        self.assertAlmostEqual(fits.hole(10.0, "close", "fdm"), 10.3)

    def test_unknown_fit_or_process_fails(self):
        with self.assertRaises(StandardError):
            fits.fit("press", "fdm")
        with self.assertRaises(StandardError):
            fits.fit("slide", "cnc")

    def test_insert_hole_only_where_the_process_takes_inserts(self):
        self.assertEqual(fits.insert_hole(M2, "mjf"), 3.2)  # the pan-tilt's
        with self.assertRaises(StandardError):
            fits.insert_hole(M3, "sla")  # resin does not take a heat-set insert
        with self.assertRaises(StandardError):
            fits.insert_hole(M2, "fdm")  # no M2 insert chosen for FDM

    def test_an_insert_hole_is_wider_than_the_screw_clearance(self):
        for p in PROCESSES.values():
            for name, ins in p.inserts.items():
                self.assertGreater(ins.hole, fasteners.screw(name).clearance)
                self.assertTrue(ins.source, (p.name, name))

    def test_unmeasured_values_say_so(self):
        """Every fit carries where it came from until a coupon measures it."""
        for p in PROCESSES.values():
            for f in p.fits.values():
                self.assertTrue(f.source, (p.name, f))


class Grid(unittest.TestCase):
    def test_points_are_on_pitch(self):
        pts = grid.points(3, 2, origin=(5.0, 5.0))
        self.assertEqual(len(pts), 6)
        self.assertEqual(pts[0], (5.0, 5.0))
        self.assertEqual(pts[-1], (25.0, 15.0))
        self.assertEqual(grid.off_grid(pts), [])

    def test_off_grid_names_the_stray_point(self):
        pts = [(0.0, 0.0), (10.0, 0.0), (20.3, 0.0)]
        self.assertEqual(grid.off_grid(pts), [(20.3, 0.0)])

    def test_off_grid_tolerates_float_noise(self):
        self.assertEqual(grid.off_grid([(0.0, 0.0), (0.1 * 3 * 100, 1e-9)]), [])

    def test_cell_plate_keeps_holes_on_pitch_across_plates(self):
        """Two n-cell plates butted end to end keep every hole on one grid."""
        size = grid.plate_size(2)
        self.assertLess(size, 2 * grid.PITCH)
        self.assertAlmostEqual(size + 2 * grid.EDGE_GAP, 2 * grid.PITCH)

    def test_grid_hole_leaves_min_wall_to_its_neighbour_and_the_edge(self):
        m3 = [p.inserts["M3"].hole for p in PROCESSES.values() if "M3" in p.inserts]
        for hole in (M3.clearance, *m3):
            self.assertGreaterEqual(grid.PITCH - hole, fits.MIN_WALL, hole)
            edge = grid.PITCH / 2 - grid.EDGE_GAP - hole / 2
            self.assertGreaterEqual(edge, fits.MIN_WALL, hole)

    def test_slot_takes_the_float_each_way(self):
        self.assertAlmostEqual(grid.slot_length(M3), M3.clearance + 2 * grid.FLOAT)

    def test_grid_screw_is_in_the_table(self):
        self.assertIs(grid.SCREW, M3)

    def test_plates_are_thick_enough_for_the_wall_gate(self):
        for t in grid.PLATES:
            self.assertGreaterEqual(t, fits.MIN_WALL)
            self.assertTrue(math.isclose(t, round(t)))


if __name__ == "__main__":
    unittest.main()
