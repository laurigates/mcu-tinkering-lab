"""kicad.fab's KiCad-free parts: the order notes printed beside the upload."""

from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
