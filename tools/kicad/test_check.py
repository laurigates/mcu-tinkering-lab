"""kicad.check's report reading: which DRC findings a pad-clipped footprint excuses.

KiCad-free: the reports and board text are synthetic, in the shape kicad-cli
10.0.7 writes them.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from kicad import check

REPORT = """** Drc report for demo.kicad_pcb **
** Report includes: Errors, Warnings, Exclusions **

** Found {n} DRC violations **
{blocks}
** Found {unconnected} unconnected pads **

** Found 0 Footprint errors **

** End of Report **
"""

MISMATCH = """[lib_footprint_mismatch]: Footprint 'SOT-23-6' does not match copy in library 'Package_TO_SOT_SMD'
    Local override; warning
    @(130.0000 mm, 163.0000 mm): Footprint {ref}
"""

SILK = """[silk_over_copper]: Silkscreen clipped by solder mask
    Local override; warning
    @(162.0000 mm, 104.0000 mm): Reference field of R2
    @(161.0000 mm, 104.0000 mm): Pad 1 [VBAT] of R2 on F.Cu
"""

BOARD = """(kicad_pcb
\t(version 20260206)
\t(footprint "Package_TO_SOT_SMD:SOT-23-6"
\t\t(property "Reference" "U10"
\t\t\t(at 0 0 0)
\t\t)
\t\t(property "Silk" "clipped 0.15 mm clear of pads (build_board.clip_silk)"
\t\t\t(at 0 0 0)
\t\t)
\t)
\t(footprint "Resistor_SMD:R_0603_1608Metric"
\t\t(property "Reference" "R2"
\t\t\t(at 0 0 0)
\t\t)
\t)
)
"""


def report(*blocks: str, unconnected: int = 0) -> str:
    return REPORT.format(n=len(blocks), blocks="".join(blocks), unconnected=unconnected)


class ClippedRefs(unittest.TestCase):
    def test_footprints_carrying_the_silk_field(self):
        self.assertEqual(check.clipped_refs(BOARD), {"U10"})


class SilkField(unittest.TestCase):
    def test_build_board_writes_the_field_check_reads(self):
        source = (Path(__file__).parent / "pcb/build_board.py").read_text()
        self.assertIn(f'SILK_FIELD = "{check.SILK_FIELD}"', source)


class Unexcused(unittest.TestCase):
    def test_a_clipped_footprints_library_mismatch_is_excused(self):
        self.assertEqual(
            check.unexcused(report(MISMATCH.format(ref="U10")), {"U10"}), []
        )

    def test_an_unclipped_footprints_mismatch_is_not(self):
        found = check.unexcused(report(MISMATCH.format(ref="R2")), {"U10"})
        self.assertEqual(len(found), 1)
        self.assertIn("Footprint R2", found[0])

    def test_any_other_violation_is_not(self):
        found = check.unexcused(report(MISMATCH.format(ref="U10"), SILK), {"U10"})
        self.assertEqual(len(found), 1)
        self.assertTrue(found[0].startswith("[silk_over_copper]"))

    def test_unconnected_pads_are_not(self):
        found = check.unexcused(report(unconnected=2), {"U10"})
        self.assertEqual(found, ["** Found 2 unconnected pads **"])

    def test_a_report_it_cannot_read_excuses_nothing(self):
        found = check.unexcused("garbage", {"U10"})
        self.assertTrue(found)


if __name__ == "__main__":
    unittest.main()
