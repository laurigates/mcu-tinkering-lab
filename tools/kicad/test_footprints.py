"""Tests for the footprint reader, against the vendored KiCad 10.0.6 subset.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from kicad import footprints, library
from kicad.errors import KicadError


def fp(fpid: str) -> Path:
    return library.footprint_file(fpid)


class FootprintTests(unittest.TestCase):
    def test_xt30_leaves_out_unnumbered_pads(self):
        f = footprints.parse(
            fp("Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal")
        )
        self.assertEqual(f.name, "AMASS_XT30PW-M_1x02_P2.50mm_Horizontal")
        self.assertEqual(list(f.pads), ["1", "2"])
        self.assertEqual((f.pads["1"].x, f.pads["1"].y), (0.0, 0.0))
        self.assertEqual((f.pads["2"].x, f.pads["2"].y), (-5.0, 0.0))
        self.assertEqual(f.attr, "through_hole")

    def test_pololu_has_16_through_hole_pads(self):
        f = footprints.parse(fp("Module:Pololu_Breakout-16_15.2x20.3mm"))
        self.assertEqual(sorted(f.pads, key=int), [str(n) for n in range(1, 17)])
        self.assertEqual({p.kind for p in f.pads.values()}, {"thru_hole"})

    def test_smd_and_mounting_hole_attr(self):
        r = footprints.parse(
            fp("Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder")
        )
        self.assertEqual((r.attr, sorted(r.pads)), ("smd", ["1", "2"]))
        self.assertEqual(r.pads["1"].shape, "roundrect")
        x0, y0, x1, y1 = r.courtyard
        self.assertTrue(x0 < 0 < x1 and y0 < 0 < y1)
        h = footprints.parse(fp("MountingHole:MountingHole_3.2mm_M3"))
        self.assertEqual(h.pads, {})
        self.assertEqual(h.attr, "exclude_from_pos_files exclude_from_bom")

    def test_thru_hole_pads_matches_parse_for_through_hole_parts(self):
        path = fp("Capacitor_THT:CP_Radial_D8.0mm_P2.50mm")
        pads, _ = footprints.thru_hole_pads(path)
        parsed = footprints.parse(path)
        self.assertEqual(pads, {n: (p.x, p.y) for n, p in parsed.pads.items()})

    def test_courtyard_is_the_box_of_the_f_crtyd_graphics_only(self):
        # (fpid, true F.CrtYd box). The first two have a silkscreen or fab
        # graphic before the courtyard, the last two a courtyard circle.
        cases = {
            "Diode_SMD:D_SOD-123": (-2.35, -1.15, 2.35, 1.15),
            "Seeed_XIAO:XIAO-RP2350-Socket": (-10.42, -8.88, 10.54, 8.87),
            "MountingHole:MountingHole_3.2mm_M3": (-3.45, -3.45, 3.45, 3.45),
            # centre (1.25, 0), radius 4.25
            "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm": (-3.0, -4.25, 5.5, 4.25),
        }
        for fpid, box in cases.items():
            with self.subTest(fpid=fpid):
                self.assertEqual(footprints.parse(fp(fpid)).courtyard, box)

    def test_thru_hole_pads_box_is_the_true_courtyard(self):
        # The perfboard footprints a regex reading of F.CrtYd got wrong: a
        # silkscreen or fab point before the courtyard leaked in (the socket's
        # x 12.59, the diode's x -1.75), or a circle read as its centre and
        # one rim point (the capacitor's +/-4.08).
        cases = {
            "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm": (-3.0, -4.25, 5.5, 4.25),
            "Seeed_XIAO:XIAO-RP2350-Socket": (-10.42, -8.88, 10.54, 8.87),
            "Diode_THT:D_DO-41_SOD81_P2.54mm_Vertical_AnodeUp": (
                -1.6,
                -1.6,
                3.89,
                1.6,
            ),
        }
        for fpid, box in cases.items():
            with self.subTest(fpid=fpid):
                self.assertEqual(footprints.thru_hole_pads(fp(fpid))[1], box)
                self.assertEqual(footprints.parse(fp(fpid)).courtyard, box)

    def test_courtyard_of_an_arc_includes_its_bulge(self):
        # A half circle of radius 2 about (0, 0), from (2, 0) through (0, 2) to
        # (-2, 0), plus a line closing it: the bulge reaches y = 2, which no
        # endpoint of either graphic does.
        text = (
            '(footprint "arc"\n'
            '\t(fp_line (start 0 -9) (end 9 -9) (layer "F.SilkS"))\n'
            '\t(fp_arc (start 2 0) (mid 0 2) (end -2 0) (layer "F.CrtYd"))\n'
            '\t(fp_line (start -2 0) (end 2 0) (layer "F.CrtYd"))\n'
            '\t(fp_rect (start -5 -5) (end 5 5) (layer "F.Fab"))\n'
            ")\n"
        )
        self.assertEqual(footprints.courtyard(text), (-2.0, 0.0, 2.0, 2.0))
        reverse = text.replace(
            "(start 2 0) (mid 0 2) (end -2 0)", "(start -2 0) (mid 0 2) (end 2 0)"
        )
        self.assertEqual(footprints.courtyard(reverse), (-2.0, 0.0, 2.0, 2.0))
        # A quarter arc across angle 0, (1, -1) -> (sqrt 2, 0) -> (1, 1): only x = sqrt 2 bulges.
        wrap = text.replace(
            "(start 2 0) (mid 0 2) (end -2 0)",
            "(start 1 -1) (mid 1.41421356 0) (end 1 1)",
        ).replace("(start -2 0) (end 2 0)", "(start 1 -1) (end 1 1)")
        box = footprints.courtyard(wrap)
        self.assertEqual((box[0], box[1], box[3]), (1.0, -1.0, 1.0))
        self.assertAlmostEqual(box[2], math.sqrt(2))
        poly = '(footprint "p"\n\t(fp_poly (pts (xy 0 0) (xy 3 1) (xy 1 4)) (layer "F.CrtYd"))\n)\n'
        self.assertEqual(footprints.courtyard(poly), (0.0, 0.0, 3.0, 4.0))

    def test_no_courtyard_raises(self):
        text = (
            '(footprint "x"\n\t(fp_line (start 0 0) (end 1 1) (layer "F.SilkS"))\n)\n'
        )
        with self.assertRaisesRegex(KicadError, "no F.CrtYd courtyard"):
            footprints.courtyard(text)

    def test_thru_hole_pads_rejects_smd(self):
        with self.assertRaises(KicadError):
            footprints.thru_hole_pads(fp("Diode_SMD:D_SOD-123"))

    def test_not_a_footprint(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.kicad_mod"
            path.write_text("(kicad_symbol_lib (version 1))\n")
            with self.assertRaises(KicadError):
                footprints.parse(path)


if __name__ == "__main__":
    unittest.main()
