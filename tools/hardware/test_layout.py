"""Tests for tools/hardware/layout.py — physical header layouts (#495).

Stdlib `unittest`, like the rest of tools/hardware:

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path

from hardware import HardwareError, board_layout, parse_board_table, parse_layout
from hardware.layout import BOARDS_DIR

REPO_ROOT = Path(__file__).resolve().parents[2]

BREAKOUT_MD = """\
    # Test breakout

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VCC | L | 1 | JP1.1 | logic |
    | GND | L | 2 | JP1.2 | |
    | SDA | R | 2 | JP2.2 | |
    | SCL | R | 1 | JP2.1 | |
    | GND | R | 3 | JP2.3 | |
    | 0 | B | 1 | JP3.1 | |
    | 1 | B | 2 | JP3.2 | |
    """

BOARD_MD = """\
    # Test MCU board

    | Board Pin | GPIO | Side | Pos | Default Function |
    |-----------|------|------|-----|------------------|
    | D0 | GPIO1 | L | 1 | x |
    | D1 | GPIO2 | L | 2 | x |
    | 5V | — | R | 1 | 5 V rail |
    | GND | — | R | 2 | Ground |
    """

# Every physical-layout reference the schematic draws from, and the pad count
# per side each one must carry — the vendor's own header, pad for pad.
BREAKOUTS = {
    "sparkfun-tb6612fng": {"L": 8, "R": 8},
    "adafruit-pca9685": {"L": 6, "R": 6, "T": 2, "B": 16},
    "adafruit-tca9548a": {"L": 12, "R": 12},
    "adafruit-max98357a": {"T": 2, "B": 7},
}


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


class ParseLayoutTest(unittest.TestCase):
    def parse(self, text: str):
        with tempfile.TemporaryDirectory() as tmp:
            return parse_layout(write(Path(tmp) / "part.md", text))

    def assertRejected(self, text: str, pattern: str):
        with tempfile.TemporaryDirectory() as tmp:
            path = write(Path(tmp) / "part.md", text)
            with self.assertRaisesRegex(HardwareError, pattern):
                parse_layout(path)

    def test_side_lists_pads_in_position_order_whatever_the_row_order(self):
        layout = self.parse(BREAKOUT_MD)
        self.assertEqual([p.name for p in layout.side("R")], ["SCL", "SDA", "GND"])
        self.assertEqual([p.name for p in layout.side("B")], ["0", "1"])
        self.assertEqual(layout.side("T"), ())

    def test_a_unique_name_is_its_own_anchor(self):
        layout = self.parse(BREAKOUT_MD)
        self.assertEqual(layout.by_anchor["SDA"].pos, 2)
        self.assertEqual(layout.by_anchor["VCC"].side, "L")

    def test_a_repeated_name_is_anchored_by_side_and_position(self):
        # Two GND pads cannot share one schematic anchor: the second would
        # silently replace the first, and a wire meant for one pad lands on
        # the other.
        layout = self.parse(BREAKOUT_MD)
        self.assertNotIn("GND", layout.by_anchor)
        self.assertEqual(layout.by_anchor["GND.L2"].name, "GND")
        self.assertEqual(layout.by_anchor["GND.R3"].name, "GND")

    def test_breakout_pads_carry_no_gpio(self):
        self.assertTrue(all(p.gpio is None for p in self.parse(BREAKOUT_MD).pads))

    def test_a_board_reference_is_read_through_the_board_parser(self):
        # The MCU's layout is the pin-mapping table's Side/Pos columns — the
        # same rows parse_board_table reads, not a second copy of them.
        with tempfile.TemporaryDirectory() as tmp:
            path = write(Path(tmp) / "board.md", BOARD_MD)
            layout = parse_layout(path)
            board = parse_board_table(path)
        self.assertEqual(
            [(p.name, p.gpio, p.side, p.pos) for p in layout.pads],
            [(p.name, p.gpio, p.side, p.pos) for p in board.pins],
        )
        self.assertEqual(layout.by_anchor["D1"].gpio, 2)

    def test_a_board_table_without_positions_is_rejected(self):
        self.assertRejected(
            """\
            | Pin | GPIO | Function |
            |-----|------|----------|
            | D0 | GPIO1 | x |
            """,
            "D0.*no physical position",
        )

    def test_a_gap_in_a_side_is_rejected(self):
        # A drawing with a missing pad cannot be counted against the board.
        self.assertRejected(
            """\
            | Pin | Side | Pos |
            |-----|------|-----|
            | A | L | 1 |
            | B | L | 3 |
            """,
            r"side L.*\[1, 3\]",
        )

    def test_two_pads_in_one_position_are_rejected(self):
        self.assertRejected(
            """\
            | Pin | Side | Pos |
            |-----|------|-----|
            | A | L | 1 |
            | B | L | 1 |
            """,
            "L 1 is claimed by both A and B",
        )

    def test_an_unknown_side_is_rejected(self):
        self.assertRejected(
            """\
            | Pin | Side | Pos |
            |-----|------|-----|
            | A | X | 1 |
            """,
            "side 'X'",
        )

    def test_a_missing_position_is_rejected(self):
        self.assertRejected(
            """\
            | Pin | Side | Pos |
            |-----|------|-----|
            | A | L | |
            """,
            "A.*position",
        )

    def test_no_layout_table_is_rejected(self):
        self.assertRejected("# nothing here\n", "no physical-layout table")

    def test_two_layout_tables_are_rejected(self):
        self.assertRejected(
            BREAKOUT_MD + "\n" + textwrap.dedent(BREAKOUT_MD),
            "2 physical-layout tables",
        )


class RealReferencesTest(unittest.TestCase):
    def test_xiao_is_seven_per_side_with_power_on_the_right(self):
        layout = board_layout("xiao-esp32s3")
        self.assertEqual(len(layout.side("L")), 7)
        self.assertEqual(len(layout.side("R")), 7)
        right = [p.name for p in layout.side("R")]
        self.assertEqual(right[:3], ["5V", "GND", "3V3"])
        # D6/D7 (GPIO43/44) are part of the header even though nothing uses them.
        self.assertIn(43, {p.gpio for p in layout.side("L")})
        self.assertIn(44, {p.gpio for p in layout.side("R")})

    def test_every_breakout_reference_parses_with_its_vendor_pad_count(self):
        for slug, counts in BREAKOUTS.items():
            with self.subTest(slug):
                layout = board_layout(slug)
                got = {s: len(layout.side(s)) for s in "LRTB" if layout.side(s)}
                self.assertEqual(got, counts)

    def test_every_breakout_reference_names_its_vendor_board_file(self):
        # The order is only as good as its source: each page must say which
        # vendor file and commit it was read from, so it can be re-derived
        # with tools/breakout-pinout.py rather than trusted.
        for slug in BREAKOUTS:
            with self.subTest(slug):
                text = (BOARDS_DIR / f"{slug}.md").read_text(encoding="utf-8")
                self.assertIn("tools/breakout-pinout.py", text)
                self.assertRegex(text, r"\.brd")
                self.assertRegex(text, r"\b[0-9a-f]{40}\b")

    def test_an_unknown_board_names_the_directory_it_looked_in(self):
        with self.assertRaisesRegex(HardwareError, "docs/reference/boards"):
            board_layout("no-such-board")


if __name__ == "__main__":
    unittest.main()
