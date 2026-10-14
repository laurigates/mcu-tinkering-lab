"""Tests for the library symbol reader, against the vendored KiCad 10.0.6 subset.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from kicad import symbols
from kicad.errors import KicadError

HERE = Path(__file__).resolve().parent

# Two units, each with a De Morgan alternate: the placed symbol is unit 1,
# style 1, plus the shared unit 0.
TWO_UNIT = """\
(kicad_symbol_lib
\t(version 20251024)
\t(generator "test")
\t(symbol "DUAL"
\t\t(in_bom no)
\t\t(symbol "DUAL_0_1"
\t\t\t(rectangle
\t\t\t\t(start -5.08 5.08)
\t\t\t\t(end 5.08 -5.08)
\t\t\t)
\t\t)
\t\t(symbol "DUAL_1_1"
\t\t\t(pin input line
\t\t\t\t(at -7.62 0 0)
\t\t\t\t(length 2.54)
\t\t\t\t(name "IN1")
\t\t\t\t(number "1")
\t\t\t)
\t\t)
\t\t(symbol "DUAL_1_2"
\t\t\t(pin input inverted
\t\t\t\t(at -7.62 2.54 0)
\t\t\t\t(length 2.54)
\t\t\t\t(name "IN1")
\t\t\t\t(number "1")
\t\t\t)
\t\t)
\t\t(symbol "DUAL_2_1"
\t\t\t(pin output line
\t\t\t\t(at 7.62 0 180)
\t\t\t\t(length 2.54)
\t\t\t\t(hide yes)
\t\t\t\t(name "OUT2")
\t\t\t\t(number "2")
\t\t\t)
\t\t)
\t)
\t(symbol "CHILD"
\t\t(extends "DUAL")
\t)
)
"""


class StockSymbolTests(unittest.TestCase):
    def test_drv8825_pins(self):
        pins = symbols.pins(symbols.load("Driver_Motor:Pololu_Breakout_DRV8825"))
        self.assertEqual(len(pins), 16)
        self.assertEqual(sorted(int(p.number) for p in pins), list(range(1, 17)))
        by_number = {p.number: p for p in pins}
        gnd1, gnd7 = by_number["1"], by_number["7"]
        self.assertEqual((gnd1.name, gnd1.etype), ("GND", "power_in"))
        self.assertEqual((gnd7.name, gnd7.etype), ("GND", "power_in"))
        self.assertEqual((gnd1.x, gnd1.y, gnd1.angle), (0.0, -20.32, 90))
        self.assertEqual((gnd7.x, gnd7.y, gnd7.angle), (2.54, -20.32, 90))
        self.assertNotEqual((gnd1.x, gnd1.y), (gnd7.x, gnd7.y))
        self.assertEqual(by_number["8"].angle, 270)  # VMOT, from the top
        self.assertEqual({p.angle for p in pins}, {0, 90, 180, 270})
        self.assertEqual({p.length for p in pins}, {2.54})
        self.assertFalse(any(p.hidden for p in pins))
        self.assertEqual(by_number["9"].name, "~{EN}")

    def test_device_r_embeds_as_kicad_did(self):
        # testdata/device-r.embedded is Device:R copied from #749's committed
        # schematic, which KiCad itself wrote.
        want = (HERE / "testdata/device-r.embedded").read_text().rstrip("\n")
        self.assertEqual(symbols.embed(symbols.load("Device:R"), "Device:R"), want)

    def test_embed_rejects_a_mismatched_id(self):
        with self.assertRaises(KicadError):
            symbols.embed(symbols.load("Device:R"), "Device:C")

    def test_flags(self):
        self.assertEqual(
            symbols.flags(symbols.load("Mechanical:MountingHole")),
            {
                "exclude_from_sim": False,
                "in_bom": False,
                "on_board": True,
                "in_pos_files": True,
            },
        )
        self.assertEqual(symbols.flags(symbols.load("Device:R"))["in_bom"], True)

    def test_passives_have_empty_pin_names(self):
        for lib_id in ("Device:R", "Device:C", "Device:C_Polarized"):
            pins = symbols.pins(symbols.load(lib_id))
            self.assertEqual([(p.number, p.name) for p in pins], [("1", ""), ("2", "")])

    def test_d_schottky_k_is_1(self):
        pins = symbols.pins(symbols.load("Device:D_Schottky"))
        self.assertEqual({p.name: p.number for p in pins}, {"K": "1", "A": "2"})

    def test_extent_of_device_r(self):
        x0, y0, x1, y1 = symbols.extent(symbols.load("Device:R"))
        self.assertEqual((y0, y1), (-3.81, 3.81))  # the two pin ends
        self.assertLess(x0, 0)
        self.assertGreater(x1, 0)

    def test_missing_symbol(self):
        with self.assertRaisesRegex(KicadError, "no symbol 'Q_NPN'"):
            symbols.load("Device:Q_NPN")


class SyntheticSymbolTests(unittest.TestCase):
    def setUp(self):
        self.block = symbols.find(TWO_UNIT, "DUAL")

    def test_unit_2_and_de_morgan_pins_are_not_placed(self):
        pins = symbols.pins(self.block)
        self.assertEqual([(p.number, p.unit, p.style) for p in pins], [("1", 1, 1)])
        every = symbols.all_pins(self.block)
        self.assertEqual(
            [(p.number, p.unit, p.style) for p in every],
            [("1", 1, 1), ("1", 1, 2), ("2", 2, 1)],
        )
        self.assertTrue(every[2].hidden)
        self.assertEqual(every[1].shape, "inverted")

    def test_extent_ignores_unplaced_units(self):
        self.assertEqual(symbols.extent(self.block), (-7.62, -5.08, 5.08, 5.08))

    def test_flags_default_when_absent(self):
        self.assertEqual(
            symbols.flags(self.block),
            {
                "exclude_from_sim": False,
                "in_bom": False,
                "on_board": True,
                "in_pos_files": True,
            },
        )

    def test_extends_is_rejected(self):
        with self.assertRaisesRegex(KicadError, "extends"):
            symbols.find(TWO_UNIT, "CHILD")

    def test_header_and_blocks(self):
        self.assertTrue(symbols.header(TWO_UNIT).endswith('(generator "test")\n'))
        self.assertEqual(list(symbols.blocks(TWO_UNIT)), ["DUAL", "CHILD"])
        with self.assertRaises(KicadError):
            symbols.header("(footprint x)")

    def test_load_through_a_vendored_tree(self):
        with tempfile.TemporaryDirectory() as tmp:
            lib = Path(tmp) / "kicad-stock/symbols/Logic.kicad_sym"
            lib.parent.mkdir(parents=True)
            lib.write_text(TWO_UNIT)
            block = symbols.load("Logic:DUAL", vendored=Path(tmp))
            self.assertEqual(block, self.block)
            self.assertTrue(
                symbols.embed(block, "Logic:DUAL").startswith(
                    '\t\t(symbol "Logic:DUAL"\n'
                )
            )


if __name__ == "__main__":
    unittest.main()
