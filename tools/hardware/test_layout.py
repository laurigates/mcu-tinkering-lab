"""Tests for tools/hardware/layout.py — physical header layouts (#495).

Stdlib `unittest`, like the rest of tools/hardware:

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path

from hardware import (
    HardwareError,
    board_layout,
    board_pad_centres,
    parse_board_table,
    parse_layout,
    parse_pad_centres,
)
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


CENTRES_MD = """\
    # Test breakout

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VCC | L | 1 | JP1.1 | |
    | GND | L | 2 | JP1.2 | |
    | OUT | T | 1 | X1.1 | |

    | Header | X (mm) | Y (mm) |
    |--------|--------|--------|
    | JP1.1 | 0.00 | -2.54 |
    | JP1.2 | 0.00 | 0.00 |
    | JP1.3 | 0.00 | 2.54 |
    | X1.1 | 12.70 | -7.62 |
    """


class ParsePadCentresTest(unittest.TestCase):
    def parse(self, text: str):
        with tempfile.TemporaryDirectory() as tmp:
            return parse_pad_centres(write(Path(tmp) / "part.md", text))

    def assertRejected(self, text: str, pattern: str):
        with tempfile.TemporaryDirectory() as tmp:
            path = write(Path(tmp) / "part.md", text)
            with self.assertRaisesRegex(HardwareError, pattern):
                parse_pad_centres(path)

    def test_centres_are_grouped_by_header_in_page_order(self):
        centres = self.parse(CENTRES_MD)
        self.assertEqual(sorted(centres), ["JP1", "X1"])
        self.assertEqual(
            [(c.pad, c.x, c.y) for c in centres["JP1"]],
            [("1", 0.0, -2.54), ("2", 0.0, 0.0), ("3", 0.0, 2.54)],
        )
        self.assertEqual(centres["X1"][0].element, "X1")

    def test_a_header_pad_the_layout_does_not_list_is_kept(self):
        # Every pad of a header gets a centre: a footprint needs JP1.3 even
        # when the layout table leaves it out (the PCA9685's V+/GND servo rows).
        self.assertIn("3", [c.pad for c in self.parse(CENTRES_MD)["JP1"]])

    def test_a_layout_pad_with_no_centre_is_rejected(self):
        self.assertRejected(
            CENTRES_MD.replace("    | X1.1 | 12.70 | -7.62 |\n", ""),
            r"X1\.1.*no pad centre",
        )

    def test_a_centre_for_a_header_the_layout_never_names_is_rejected(self):
        self.assertRejected(
            CENTRES_MD + "    | JP9.1 | 1.00 | 1.00 |\n",
            "JP9.*not a header the layout table names",
        )

    def test_a_repeated_pad_is_rejected(self):
        self.assertRejected(
            CENTRES_MD + "    | JP1.1 | 5.00 | 5.00 |\n",
            r"JP1\.1.*twice",
        )

    def test_a_header_cell_without_element_and_pad_is_rejected(self):
        self.assertRejected(
            CENTRES_MD.replace("| JP1.3 |", "| JP13 |"),
            "'JP13'.*<element>.<pad>",
        )

    def test_a_non_numeric_coordinate_is_rejected(self):
        self.assertRejected(
            CENTRES_MD.replace("| 12.70 | -7.62 |", "| 12.70 | nan |"),
            r"X1\.1.*'nan'",
        )

    def test_no_centre_table_is_rejected(self):
        layout_only = CENTRES_MD.split("    | Header |")[0]
        self.assertRejected(layout_only, "no pad-centre table")

    def test_two_centre_tables_are_rejected(self):
        table = "    | Header |" + CENTRES_MD.split("    | Header |")[1]
        self.assertRejected(CENTRES_MD + "\n" + table, "2 pad-centre tables")

    def test_the_centre_table_is_not_a_layout_table(self):
        # Adding the table must leave the layout reader's one-table rule intact.
        with tempfile.TemporaryDirectory() as tmp:
            layout = parse_layout(write(Path(tmp) / "part.md", CENTRES_MD))
        self.assertEqual(len(layout.pads), 3)


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

    def test_every_breakout_reference_records_its_pad_centres(self):
        # Every pad of every header the layout table cites: two 1x08 rows on
        # the TB6612, two 1x12 on the TCA9548A, a 1x07 and the terminal on the
        # MAX98357A, and on the PCA9685 the terminal, both side headers and
        # all four 3x04 servo blocks.
        expected = {
            "sparkfun-tb6612fng": {"JP1": 8, "JP2": 8},
            "adafruit-pca9685": {
                "J1": 2,
                "JP1": 12,
                "JP2": 12,
                "JP3": 6,
                "JP4": 6,
                "JP5": 12,
                "JP6": 12,
            },
            "adafruit-tca9548a": {"JP1": 12, "JP3": 12},
            "adafruit-max98357a": {"JP1": 7, "X1": 2},
        }
        for slug, counts in expected.items():
            with self.subTest(slug):
                centres = board_pad_centres(slug)
                self.assertEqual({h: len(p) for h, p in centres.items()}, counts)

    def test_pad_centres_are_in_kicad_y_down_convention(self):
        # The PCA9685 page's top view runs JP3 from GND (pad 6) at the top to
        # V+ (pad 1) at the bottom. Eagle stores pad 6 at the larger y; after
        # the flip it must have the SMALLER y.
        pca = board_pad_centres("adafruit-pca9685")
        jp3 = {c.pad: c for c in pca["JP3"]}
        jp4 = {c.pad: c for c in pca["JP4"]}
        self.assertLess(jp3["6"].y, jp3["1"].y)
        self.assertAlmostEqual(jp3["1"].y - jp3["6"].y, 12.70)
        self.assertAlmostEqual(jp4["1"].x - jp3["1"].x, 58.42)
        # TB6612: VM (JP1.8) and PWMA (JP2.1) are the two top corners, 15.24 mm
        # apart, and JP1 runs downward from VM to GND (pad 1).
        tb = board_pad_centres("sparkfun-tb6612fng")
        jp1 = {c.pad: c for c in tb["JP1"]}
        jp2 = {c.pad: c for c in tb["JP2"]}
        self.assertAlmostEqual(jp1["8"].y, jp2["1"].y)
        self.assertAlmostEqual(jp2["1"].x - jp1["8"].x, 15.24)
        self.assertLess(jp1["8"].y, jp1["1"].y)

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
