"""Tests for the footprint reader, against the vendored KiCad 10.0.6 subset.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

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
        pads, box = footprints.thru_hole_pads(path)
        parsed = footprints.parse(path)
        self.assertEqual(pads, {n: (p.x, p.y) for n, p in parsed.pads.items()})
        self.assertEqual(box, parsed.courtyard)

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
