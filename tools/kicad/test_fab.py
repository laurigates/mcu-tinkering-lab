"""kicad.fab's KiCad-free parts: the order notes printed beside the upload, the
previous run's outputs it deletes, and the DRC items it fails on."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from kicad import fab, paths


def cfg(data: dict) -> paths.BoardConfig:
    return paths.BoardConfig(Path("board.toml"), "demo", "A4", data)


class OrderNotes(unittest.TestCase):
    def test_standard_silkscreen_needs_no_note(self):
        self.assertEqual(fab.order_notes(cfg({})), [])
        self.assertEqual(fab.order_notes(cfg({"silk": {"high_precision": False}})), [])

    def test_high_precision_silkscreen_is_an_order_option(self):
        (note,) = fab.order_notes(cfg({"silk": {"high_precision": True}}))
        self.assertIn("high-precision", note)
        self.assertIn("0.8 mm", note)


class StaleOutputs(unittest.TestCase):
    def test_previous_zips_and_handoff_rows_are_found(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            names = [
                "demo-fab.zip",
                "demo-fab-0123456.zip",
                "demo-fab-0123456.csv",
                "demo-fab-abcdef0-dirty.zip",
                "other-fab-0123456.zip",
                "demo-fab-notes.txt",
            ]
            for n in names:
                (out / n).write_text("x")
            (out / "fab").mkdir()
            found = [p.name for p in fab.stale_outputs(out, "demo")]
        self.assertEqual(
            found,
            [
                "demo-fab-0123456.csv",
                "demo-fab-0123456.zip",
                "demo-fab-abcdef0-dirty.zip",
                "demo-fab.zip",
            ],
        )

    def test_no_output_dir_yet(self):
        self.assertEqual(fab.stale_outputs(Path("/nonexistent/output"), "demo"), [])


# Shaped like kicad-cli 10's DRC report (2026-11-09, balancebot, the text put
# on J3's pins on purpose).
REPORT = """** Found 3 DRC violations **
[lib_footprint_issues]: The current configuration does not include the footprint library 'Seeed_XIAO'
    Local override; warning
    @(145.0000 mm, 114.0000 mm): Footprint U1
[silk_overlap]: Silkscreen clearance
    Rule: board setup constraints silk; warning
    @(136.1100 mm, 147.0000 mm): PCB text 'a1b2c3d 1a2b3c4d' on F.Silkscreen
    @(134.5000 mm, 131.5000 mm): Rectangle on F.Silkscreen
[silk_over_copper]: Silkscreen clipped by solder mask (rule 'silkscreen 0.15 mm clear of pads' clearance 0.1500 mm; actual 0.0000 mm)
    Rule: silkscreen 0.15 mm clear of pads; warning
    @(136.1100 mm, 147.0000 mm): PCB text 'a1b2c3d 1a2b3c4d' on F.Silkscreen
    @(136.1100 mm, 147.0000 mm): PTH pad 1 [+3V3] of J3

** Found 0 unconnected pads **
"""  # noqa: E501


class ProvenanceDrc(unittest.TestCase):
    def test_items_naming_the_text_are_returned_alone(self):
        found = fab.provenance_violations(REPORT, "a1b2c3d 1a2b3c4d")
        self.assertEqual(len(found), 2)
        self.assertTrue(found[0].startswith("[silk_overlap]"))
        self.assertTrue(found[1].endswith("PTH pad 1 [+3V3] of J3"))

    def test_other_items_do_not_count(self):
        self.assertEqual(fab.provenance_violations(REPORT, "0000000 00000000"), [])


if __name__ == "__main__":
    unittest.main()
