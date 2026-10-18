"""Tests for the courtyard areas `kicad.area` reports.

Built from the frozen balancebot inputs (`kicad.testfixture`) and synthetic
parts, never a live project's totals: a placement or part change must not need
a test edit.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from hardware import MCU

from kicad import area, footprints, library, netlist, testfixture

ROOT, PROJECT = testfixture.shared()


def comp(ref: str, key: str) -> netlist.Component:
    return netlist.Component(
        ref=ref,
        value="v",
        footprint="Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
        lib_id="Device:R",
        fields=(),
        in_bom=True,
        on_board=True,
        part_key=key,
        registry="r0805",
    )


def part(ref: str, fp: str, w: float, h: float, group: str = area.MAIN) -> area.Part:
    return area.Part(ref, "v", fp, group, w, h, smd=True, pads=2)


class GroupTests(unittest.TestCase):
    def test_head_parts(self):
        on = {"U1", "U3", "J2", "J16"}
        for ref, key in (
            ("U1", MCU),
            ("U3", f"{MCU}.head2"),
            ("J2", netlist.HEAD),
            ("J16", f"{netlist.HEAD}2"),
        ):
            with self.subTest(key=key):
                self.assertEqual(area.group_of(comp(ref, key), on), area.HEAD)

    def test_without_a_cable_the_mcu_is_on_the_main_board(self):
        self.assertEqual(area.group_of(comp("U1", MCU), {"U1"}, False), area.MAIN)

    def test_main_end_of_the_cable_is_on_the_main_board(self):
        self.assertEqual(area.group_of(comp("J1", netlist.MAIN), {"J1"}), area.MAIN)

    def test_part_on_no_net_is_mechanical(self):
        self.assertEqual(area.group_of(comp("H1", "hole"), {"R1"}), area.MECHANICAL)
        self.assertEqual(area.group_of(comp("R1", "pull"), {"R1"}), area.MAIN)


class MeasureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT, ROOT)
        cls.found = area.parts(cls.nl)

    def test_every_part_is_measured_once(self):
        self.assertEqual([p.ref for p in self.found], [c.ref for c in self.nl.comps])

    def test_size_is_the_footprint_courtyard(self):
        for p in self.found:
            with self.subTest(ref=p.ref):
                fp = footprints.parse(library.footprint_file(p.footprint))
                x0, y0, x1, y1 = fp.courtyard
                self.assertAlmostEqual(p.area, (x1 - x0) * (y1 - y0))
                self.assertEqual(p.pads, len(fp.pads))

    def test_mounting_holes_are_mechanical(self):
        holes = [p for p in self.found if p.ref.startswith("H")]
        self.assertTrue(holes)
        self.assertTrue(all(p.group == area.MECHANICAL for p in holes))
        others = [p for p in self.found if not p.ref.startswith("H")]
        self.assertTrue(all(p.group == area.MAIN for p in others))


class CompareTests(unittest.TestCase):
    def test_parts_are_matched_by_ref_and_footprint(self):
        a = [part("R1", "R", 2, 1), part("A1", "Socket", 10, 10)]
        b = [part("R1", "R", 2, 1), part("U5", "TSSOP", 3, 3), part("A1", "QFN", 1, 1)]
        lines = area.compare(a, b, "a", "b")
        self.assertEqual(lines[0], "main board: 1 parts in both, 2.0 mm²")
        self.assertEqual(lines[1], "only in a: 1 parts, 100.0 mm²")
        self.assertIn("only in b: 2 parts, 10.0 mm²", lines)
        self.assertEqual(lines[-1], "main board courtyards: 102.0 -> 12.0 mm² (-88%)")

    def test_other_groups_are_left_out(self):
        a = [part("U1", "XIAO", 20, 20, area.HEAD), part("R1", "R", 2, 1)]
        b = [part("R1", "R", 2, 1)]
        lines = area.compare(a, b, "a", "b")
        self.assertIn("only in a: 0 parts, 0.0 mm²", lines)


if __name__ == "__main__":
    unittest.main()
