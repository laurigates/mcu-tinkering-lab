"""Tests for the perfboard hole grid: rotation, snapping and hole names.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from kicad.errors import KicadError
from kicad.perfboard.grid import col_name, hole_name, overlaps, rotate, to_hole


class GridTests(unittest.TestCase):
    def test_column_names_continue_past_z(self):
        cases = {0: "A", 17: "R", 25: "Z", 26: "AA", 27: "AB", 51: "AZ", 52: "BA"}
        cases |= {701: "ZZ", 702: "AAA"}
        for c, name in cases.items():
            with self.subTest(c=c):
                self.assertEqual(col_name(c), name)

    def test_hole_names(self):
        self.assertEqual(hole_name(0, 0), "A1")
        self.assertEqual(hole_name(8, 12), "I13")
        self.assertEqual(hole_name(26, 4), "AA5")
        # Only an off-the-board error names a hole left of column A.
        self.assertEqual(hole_name(-1, 2), "[-1]3")

    def test_rotate_is_kicads(self):
        self.assertEqual(rotate(1, 2, 0), (1, 2))
        self.assertEqual(rotate(1, 2, 90), (2, -1))
        self.assertEqual(rotate(1, 2, 180), (-1, -2))
        self.assertEqual(rotate(1, 2, 270), (-2, 1))
        self.assertEqual(rotate(1, 2, 450), (2, -1))

    def test_to_hole(self):
        self.assertEqual(to_hole(5.08, "x"), 2)
        self.assertEqual(to_hole(2.50, "x"), 1)  # 2.50 mm leads bend onto 2.54
        self.assertEqual(to_hole(-7.62, "x"), -3)
        with self.assertRaisesRegex(
            KicadError, r"J9\.2: 1\.27 mm is not on the 2\.54 mm grid"
        ):
            to_hole(1.27, "J9.2")

    def test_overlaps(self):
        self.assertTrue(overlaps((0, 0, 2, 2), (1, 1, 3, 3)))
        self.assertFalse(overlaps((0, 0, 2, 2), (2, 0, 4, 2)))  # touching edges
        self.assertFalse(overlaps((0, 0, 2, 2), (0, 3, 2, 4)))


if __name__ == "__main__":
    unittest.main()
