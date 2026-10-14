"""Tests for the parts registry, docs/reference/parts/ (format: its README.md).

Every committed entry must validate against the vendored library. Each
mutation runs on a temp copy of one entry and must fail naming its cause; the
polarity tripwire cases are the ones that would otherwise produce a board on
which netlist, layout and DRC all agree with a reversed part.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import math
import re
import shutil
import tempfile
import unittest
from pathlib import Path

from hardware import parse_board_table

from kicad import footprints, library, registry, sexpr, symbols
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

PARTS = REPO_ROOT / registry.REGISTRY_REL
XIAO_BOARD = REPO_ROOT / "docs/reference/boards/xiao-rp2350.md"

# Two units: the writer places unit 1 only, so unit 2's pin would get no stub.
DUAL = """\
(kicad_symbol_lib
\t(version 20251024)
\t(generator "test")
\t(symbol "DUAL"
\t\t(symbol "DUAL_1_1"
\t\t\t(pin passive line
\t\t\t\t(at -7.62 0 0)
\t\t\t\t(length 2.54)
\t\t\t\t(name "IN1")
\t\t\t\t(number "1")
\t\t\t)
\t\t)
\t\t(symbol "DUAL_2_1"
\t\t\t(pin passive line
\t\t\t\t(at 7.62 0 180)
\t\t\t\t(length 2.54)
\t\t\t\t(name "OUT2")
\t\t\t\t(number "2")
\t\t\t)
\t\t)
\t)
)
"""


def text(slug: str) -> str:
    return (PARTS / f"{slug}.toml").read_text(encoding="utf-8")


class CommittedRegistryTests(unittest.TestCase):
    def test_every_entry_validates(self):
        specs = registry.load_all()
        self.assertGreaterEqual(len(specs), 11)
        for spec in specs:
            with self.subTest(slug=spec.slug):
                registry.validate(spec)

    def test_the_balancebot_entries(self):
        drv = registry.load("pololu-drv8825")
        self.assertEqual(drv.pins["GND"], ("1", "7"))
        self.assertEqual(drv.no_connect, ("FAULT",))
        self.assertEqual(drv.perfboard.height, "raised")
        self.assertEqual(drv.perfboard.footprint, drv.footprint)
        gy = registry.load("gy-521")
        self.assertEqual(gy.perfboard.body, (-0.6, -0.6, 5.7, 7.6))
        diode = registry.load("b5819w")
        self.assertEqual((diode.value, diode.lcsc), ("B5819W", "C8598"))
        self.assertEqual(diode.perfboard.value, "1N5819")
        hole = registry.load("m3-mounting-hole")
        self.assertEqual(hole.pins, {})
        xiao = registry.load("seeed-xiao-rp2350-socket")
        self.assertEqual((xiao.pin_numbers, xiao.pins), ("board-dip-ccw", {}))

    def test_every_entry_says_where_its_pin_order_came_from(self):
        for spec in registry.load_all():
            with self.subTest(slug=spec.slug):
                self.assertTrue(spec.source.strip())

    def test_polarised_names(self):
        self.assertEqual(registry.polarised(registry.load("r0805")), [])
        self.assertEqual(registry.polarised(registry.load("b5819w")), ["K", "A"])
        self.assertEqual(registry.polarised(registry.load("battery-xt30")), ["+", "-"])

    def test_c_polarized_convention_matches_the_symbol_graphic(self):
        """CONVENTIONS' "+" pin is the pin nearest the "+" drawn in the symbol."""
        block = symbols.load("Device:C_Polarized")
        segments = []
        for _, _, unit in symbols._drawn(block):
            for item in sexpr.children(unit):
                if sexpr.head(item)[0] != "polyline":
                    continue
                pts = [
                    (float(x), float(y))
                    for x, y in re.findall(r"\(xy (-?[\d.]+) (-?[\d.]+)\)", item)
                ]
                if len(pts) == 2:
                    segments.append(pts)
        horizontal = [s for s in segments if s[0][1] == s[1][1]]
        vertical = [s for s in segments if s[0][0] == s[1][0]]
        crosses = [
            ((h[0][0] + h[1][0]) / 2, h[0][1])
            for h in horizontal
            for v in vertical
            if math.isclose((h[0][0] + h[1][0]) / 2, v[0][0])
            and math.isclose((v[0][1] + v[1][1]) / 2, h[0][1])
        ]
        self.assertEqual(len(crosses), 1, "expected exactly one '+' mark")
        x, y = crosses[0]
        nearest = min(symbols.pins(block), key=lambda p: math.hypot(p.x - x, p.y - y))
        self.assertEqual(
            nearest.number, registry.CONVENTIONS["Device:C_Polarized"]["+"]
        )


class PinMapTests(unittest.TestCase):
    def setUp(self):
        self.spec = registry.load("seeed-xiao-rp2350-socket")
        self.pins = symbols.pins(symbols.load(self.spec.symbol))

    def test_board_dip_ccw(self):
        mapping = registry.pin_map(self.spec, parse_board_table(XIAO_BOARD), self.pins)
        self.assertEqual(mapping["D0"], ("1",))
        self.assertEqual(mapping["D6"], ("7",))
        self.assertEqual(mapping["D7"], ("8",))
        self.assertEqual(mapping["3V3"], ("12",))
        self.assertEqual(mapping["GND"], ("13",))
        self.assertEqual(mapping["5V"], ("14",))

    def test_a_mirrored_board_page_fails_the_pin_name_rule(self):
        page = XIAO_BOARD.read_text(encoding="utf-8")
        mirrored = re.sub(
            r"\| ([LR]) \|", lambda m: "| R |" if m.group(1) == "L" else "| L |", page
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "xiao.md"
            path.write_text(mirrored, encoding="utf-8")
            with self.assertRaisesRegex(KicadError, "names pin"):
                registry.pin_map(self.spec, parse_board_table(path), self.pins)

    def test_needs_a_board(self):
        with self.assertRaisesRegex(KicadError, "needs a board page"):
            registry.pin_map(self.spec, None, self.pins)

    def test_may_float(self):
        board = parse_board_table(XIAO_BOARD)
        self.assertTrue(registry.may_float(self.spec, board, "D1"))
        for power in ("GND", "5V", "3V3"):
            self.assertFalse(registry.may_float(self.spec, board, power), power)
        drv = registry.load("pololu-drv8825")
        self.assertTrue(registry.may_float(drv, None, "FAULT"))
        self.assertFalse(registry.may_float(drv, None, "STEP"))


class MutationTests(unittest.TestCase):
    """Each edit of a temp copy of an entry must fail, naming the cause."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root / registry.REGISTRY_REL).mkdir(parents=True)

    def spec(self, slug: str, body: str) -> registry.PartSpec:
        registry.entry_path(slug, self.root).write_text(body, encoding="utf-8")
        return registry.load(slug, self.root)

    def fails(
        self, slug: str, body: str, msg: str, vendored: Path = library.VENDORED
    ) -> None:
        with self.assertRaisesRegex(KicadError, msg):
            registry.validate(self.spec(slug, body), vendored)

    def test_unedited_copy_passes(self):
        for slug in registry.slugs():
            registry.validate(self.spec(slug, text(slug)))

    def test_missing_pin_number(self):
        self.fails(
            "pololu-drv8825",
            text("pololu-drv8825").replace("DIR = 16\n", ""),
            r"symbol pin\(s\) \['16'\] .* not mapped",
        )

    def test_duplicate_pin_number(self):
        self.fails(
            "pololu-drv8825",
            text("pololu-drv8825").replace("DIR = 16", "DIR = 15"),
            "pin 15 is mapped by both 'STEP' and 'DIR'",
        )

    def test_unknown_pin_number(self):
        self.fails(
            "r0805",
            text("r0805").replace("B = 2", "B = [2, 3]"),
            r"has no pin\(s\) \['3'\]",
        )

    def test_pin_with_no_footprint_pad(self):
        body = text("r0805").replace(
            '"Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"',
            '"MountingHole:MountingHole_3.2mm_M3"',
        )
        self.fails(
            "r0805",
            body,
            r"\[part\] footprint .* has no pad for symbol pin\(s\) \['1', '2'\]",
        )

    def test_unknown_no_connect_name(self):
        with self.assertRaisesRegex(
            KicadError, r"no_connect \['NOPE'\] not in \[pins\]"
        ):
            self.spec(
                "gy-521", text("gy-521").replace('["XDA", "XCL"]', '["XDA", "NOPE"]')
            )

    def test_pin_numbers_with_pins(self):
        body = text("seeed-xiao-rp2350-socket") + "\n[pins]\nD0 = 1\n"
        with self.assertRaisesRegex(KicadError, "mutually exclusive"):
            self.spec("seeed-xiao-rp2350-socket", body)

    def test_unvendored_symbol(self):
        self.fails(
            "r0805",
            text("r0805").replace('"Device:R"', '"Device:L"'),
            "no symbol 'L'|not vendored",
        )

    def test_unvendored_footprint(self):
        body = text("r0805").replace("Pad1.20x1.40mm_HandSolder", "Pad1.30x1.40mm")
        self.fails("r0805", body, "not vendored.*just kicad::vendor")

    def test_multi_unit_symbol(self):
        vendored = self.root / "lib"
        shutil.copytree(library.VENDORED, vendored)
        (vendored / "kicad-stock/symbols/Test.kicad_sym").write_text(
            DUAL, encoding="utf-8"
        )
        body = (
            text("r0805")
            .replace('"Device:R"', '"Test:DUAL"')
            .replace("A = 1\nB = 2", "IN1 = 1\nOUT2 = 2")
        )
        self.fails("r0805", body, "unit >= 2 or body style 2", vendored)

    def test_no_connect_type_pin_must_be_listed(self):
        vendored = self.root / "lib"
        shutil.copytree(library.VENDORED, vendored)
        nc = DUAL.replace(
            '\t\t(symbol "DUAL_2_1"\n\t\t\t(pin passive',
            '\t\t(symbol "DUAL_1_0"\n\t\t\t(pin no_connect',
        )
        nc = nc.replace('(symbol "DUAL_1_0"', '(symbol "DUAL_0_1"')
        (vendored / "kicad-stock/symbols/Test.kicad_sym").write_text(
            nc, encoding="utf-8"
        )
        body = (
            text("r0805")
            .replace('"Device:R"', '"Test:DUAL"')
            .replace("A = 1\nB = 2", "IN1 = 1\nOUT2 = 2")
        )
        self.fails("r0805", body, "'OUT2' is a no_connect pin", vendored)

    def test_unknown_keys(self):
        cases = {
            "[part]": text("r0805").replace("[part]\n", "[part]\ncolour = 'red'\n"),
            "top": text("r0805") + "\n[extra]\n",
            "[perfboard]": text("r0805") + 'colour = "red"\n',
        }
        for where, body in cases.items():
            with (
                self.subTest(where=where),
                self.assertRaisesRegex(KicadError, "unknown"),
            ):
                self.spec("r0805", body)

    def test_bad_values(self):
        cases = {
            'height = "raised"': ('height = "huge"', "height 'huge'"),
            "body = [-0.6, -0.6, 5.7, 7.6]": (
                "body = [1, 2, 3]",
                "body must be 4 numbers",
            ),
            'source = "': (
                'lcsc = "17414"\nsource = "',
                "lcsc '17414'",
            ),
        }
        for old, (new, msg) in cases.items():
            with self.subTest(new=new), self.assertRaisesRegex(KicadError, msg):
                self.spec("gy-521", text("gy-521").replace(old, new))
        with self.assertRaisesRegex(KicadError, "pin_numbers 'dip'"):
            self.spec(
                "seeed-xiao-rp2350-socket",
                text("seeed-xiao-rp2350-socket").replace('"board-dip-ccw"', '"dip"'),
            )
        with self.assertRaisesRegex(KicadError, r"\[part\] is required"):
            self.spec("r0805", "[pins]\n")
        with self.assertRaisesRegex(KicadError, r"\[pins\] is required"):
            self.spec("r0805", text("r0805").split("[pins]")[0])
        with self.assertRaisesRegex(KicadError, "no parts-registry entry"):
            registry.load("no-such-part", self.root)
        with self.assertRaisesRegex(KicadError, "not a parts-registry slug"):
            registry.load("../r0805", self.root)

    def test_perfboard_footprint_missing_a_symbol_pin(self):
        body = text("pololu-drv8825").replace(
            '[perfboard]\nheight = "raised"',
            '[perfboard]\nfootprint = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"\nheight = "raised"',
        )
        self.fails(
            "pololu-drv8825",
            body,
            r"\[perfboard\] footprint .* no pad for symbol pin\(s\) \['5'",
        )

    # Polarity tripwire: each must fail, from its own proof.

    def test_swapped_xt30_fails_on_the_footprint_text(self):
        body = text("battery-xt30").replace('"-" = 1\n"+" = 2', '"-" = 2\n"+" = 1')
        self.assertNotEqual(body, text("battery-xt30"))
        self.fails("battery-xt30", body, r"prints '[+-]' beside pad")

    def test_swapped_schottky_fails_on_the_pin_name_rule(self):
        body = text("b5819w").replace("K = 1\nA = 2", "K = 2\nA = 1")
        self.fails(
            "b5819w",
            body,
            r"'K' maps to \['2'\], but Device:D_Schottky names pin\(s\) \['1'\] 'K'",
        )

    def test_swapped_electrolytic_fails_on_the_convention(self):
        body = text("cp-elec-6.3x7.7").replace('"+" = 1\n"-" = 2', '"+" = 2\n"-" = 1')
        self.fails(
            "cp-elec-6.3x7.7",
            body,
            r"Device:C_Polarized's '\+' is pin 1 \(registry.CONVENTIONS\)",
        )

    def test_polarised_entry_with_no_proof_fails(self):
        body = """\
[part]
name = "Polarised header"
source = "test"
symbol = "Connector_Generic:Conn_01x02"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"

[pins]
"+" = 1
"-" = 2
"""
        self.fails("polarised-header", body, r"'\+': polarity unproven")

    def test_stand_in_footprint_marks_must_agree(self):
        """A [perfboard] stand-in whose own "A" sits by the cathode pad fails."""
        vendored = self.root / "lib"
        shutil.copytree(library.VENDORED, vendored)
        do41 = (
            vendored
            / "kicad-stock/footprints/Diode_THT.pretty/D_DO-41_SOD81_P2.54mm_Vertical_AnodeUp.kicad_mod"
        )
        src = do41.read_text(encoding="utf-8")
        self.assertEqual(src.count("(at 4.64 0 0)"), 2)
        do41.write_text(src.replace("(at 4.64 0 0)", "(at -2.1 0 0)"), encoding="utf-8")
        self.fails(
            "b5819w", text("b5819w"), r"D_DO-41.* prints 'A' beside pad 1", vendored
        )


class UserTextTests(unittest.TestCase):
    def test_xt30_marks(self):
        path = library.footprint_file(
            "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal"
        )
        marks = {t: (x, y) for t, x, y in footprints.user_texts(path)}
        self.assertEqual(marks["+"], (-8.0, 0.0))
        self.assertEqual(marks["-"], (3.0, 0.0))
        self.assertIn("${REFERENCE}", marks)


if __name__ == "__main__":
    unittest.main()
