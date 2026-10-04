"""Tests for tools/hardware — the board × header × parts join (ADR-021, #459).

Stdlib `unittest` only, so the suite runs wherever the generator does (CI's
system python3, the justfile, the pre-commit hook) with nothing to install:

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

from hardware import (
    HardwareError,
    join,
    parse_board_table,
    parse_defines,
    roles_from_defines,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
UNIFIED = REPO_ROOT / "packages/robocar/unified"
XIAO_S3 = REPO_ROOT / "docs/reference/boards/xiao-esp32s3.md"
GENERATOR = REPO_ROOT / "tools/typst/generate-pin-defs.py"


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


BOARD_MD = """\
    # Test board

    | Board Pin | GPIO | Side | Pos | Default Function | Alternate Functions |
    |-----------|------|------|-----|------------------|---------------------|
    | D0 | GPIO1 | L | 1 | Analog input | ADC1_CH0 |
    | D1 | GPIO3 | L | 2 | Analog input | Touch3 (strapping) |
    | 5V | — | R | 1 | 5 V rail | — |
    | D2 | GPIO7 | R | 2 | SPI SCK | — |
    """

HEADER_H = """\
    #define LED_PIN GPIO_NUM_1   // D0
    #define BEEP_PIN GPIO_NUM_7  // D2
    #define BUS_FREQ_HZ 400000
    """


class ParseDefinesTest(unittest.TestCase):
    def test_reads_value_and_drops_trailing_comment(self):
        with tempfile.TemporaryDirectory() as tmp:
            h = write(Path(tmp) / "a.h", "#define FOO 0x40  // the address\n")
            self.assertEqual(parse_defines([h]), {"FOO": "0x40"})

    def test_identical_redefinition_across_headers_is_allowed(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = write(Path(tmp) / "a.h", "#define FOO 1\n")
            b = write(Path(tmp) / "b.h", "#define FOO 1\n")
            self.assertEqual(parse_defines([a, b]), {"FOO": "1"})

    def test_a_valueless_define_does_not_swallow_the_next_line(self):
        # An include guard right above a #define used to take that whole
        # line as its value, dropping the next macro from the result.
        with tempfile.TemporaryDirectory() as tmp:
            h = write(
                Path(tmp) / "a.h",
                "#define A_H\n#define I2C_SDA_PIN GPIO_NUM_5\n",
            )
            self.assertEqual(parse_defines([h]), {"I2C_SDA_PIN": "GPIO_NUM_5"})

    def test_conflicting_redefinition_is_an_error(self):
        # Output would otherwise depend on argument order.
        with tempfile.TemporaryDirectory() as tmp:
            a = write(Path(tmp) / "a.h", "#define FOO 1\n")
            b = write(Path(tmp) / "b.h", "#define FOO 2\n")
            with self.assertRaisesRegex(HardwareError, "FOO"):
                parse_defines([a, b])


class RobocarConventionTest(unittest.TestCase):
    def test_only_gpio_num_macros_are_roles(self):
        defines = {
            "I2C_SDA_PIN": "GPIO_NUM_5",
            "I2C_MASTER_FREQ_HZ": "400000",
            "PCA9685_ADDR": "0x40",
            "MOTOR_LEFT_PWM_CHANNEL": "13",
        }
        self.assertEqual(roles_from_defines(defines, "robocar"), {"I2C_SDA_PIN": 5})

    def test_unknown_convention_is_an_error_naming_the_known_ones(self):
        with self.assertRaisesRegex(HardwareError, "robocar"):
            roles_from_defines({}, "balancebot")


class BoardTableTest(unittest.TestCase):
    def parse(self, text: str):
        with tempfile.TemporaryDirectory() as tmp:
            return parse_board_table(write(Path(tmp) / "board.md", text))

    def test_parses_rows_gpio_side_and_position(self):
        board = self.parse(BOARD_MD)
        d0 = board.by_name["D0"]
        self.assertEqual((d0.gpio, d0.side, d0.pos), (1, "L", 1))
        self.assertIs(board.by_gpio[7], board.by_name["D2"])

    def test_power_pin_has_no_gpio_and_is_not_in_the_gpio_index(self):
        board = self.parse(BOARD_MD)
        self.assertIsNone(board.by_name["5V"].gpio)
        self.assertEqual(sorted(board.by_gpio), [1, 3, 7])

    def test_strapping_is_read_from_the_function_columns(self):
        board = self.parse(BOARD_MD)
        self.assertTrue(board.by_name["D1"].strapping)
        self.assertFalse(board.by_name["D0"].strapping)

    def test_side_and_position_are_optional_columns(self):
        board = self.parse("""\
            | Pin | GPIO | Function |
            |-----|------|----------|
            | D0 | GPIO1 | x |
            """)
        self.assertEqual(
            (board.by_name["D0"].side, board.by_name["D0"].pos), (None, None)
        )

    def test_a_table_whose_first_column_is_not_a_pin_is_ignored(self):
        # The "Pins to Use with Caution" table has a GPIO column but is keyed
        # by GPIO, not by board pin, and must not be mistaken for the mapping.
        board = self.parse(
            BOARD_MD
            + """
            | GPIO | Issue |
            |------|-------|
            | GPIO3 | Strapping pin |
            """
        )
        self.assertEqual(len(board.pins), 4)

    def test_no_mapping_table_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "no pin-mapping table"):
            self.parse("# nothing here\n")

    def test_two_mapping_tables_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "2 pin-mapping tables"):
            self.parse(BOARD_MD + "\n" + BOARD_MD)

    def test_an_unreadable_gpio_cell_is_an_error_not_a_skip(self):
        with self.assertRaisesRegex(HardwareError, "GPIOx"):
            self.parse("""\
                | Pin | GPIO |
                |-----|------|
                | D0 | GPIOx |
                """)

    def test_two_pads_claiming_one_physical_position_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "L 1"):
            self.parse("""\
                | Pin | GPIO | Side | Pos |
                |-----|------|------|-----|
                | D0 | GPIO1 | L | 1 |
                | D1 | GPIO2 | L | 1 |
                """)

    def test_two_pads_claiming_one_gpio_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "GPIO1"):
            self.parse("""\
                | Pin | GPIO |
                |-----|------|
                | D0 | GPIO1 |
                | D1 | GPIO1 |
                """)


class XiaoEsp32s3ReferenceTest(unittest.TestCase):
    """The real board reference the robocar-unified join reads."""

    @classmethod
    def setUpClass(cls):
        cls.board = parse_board_table(XIAO_S3)

    def test_all_fourteen_header_positions_are_listed(self):
        self.assertEqual(len(self.board.pins), 14)

    def test_each_side_is_positions_one_to_seven(self):
        for side in ("L", "R"):
            positions = sorted(p.pos for p in self.board.pins if p.side == side)
            self.assertEqual(positions, list(range(1, 8)), side)

    def test_silkscreen_to_gpio(self):
        # Spot checks against Seeed's KiCad symbol, not against this file.
        expect = {"D0": 1, "D4": 5, "D5": 6, "D6": 43, "D7": 44, "D8": 7, "D10": 9}
        for name, gpio in expect.items():
            self.assertEqual(self.board.by_name[name].gpio, gpio, name)

    def test_physical_order_matches_seeeds_footprint(self):
        # Seeed's XIAO-ESP32-S3 footprint puts pad 1 (D0) and pad 14 (VBUS/5V)
        # nearest the USB-C connector, on opposite sides; pad 7 (D6) and pad 8
        # (D7) at the far end.
        by_name = self.board.by_name
        self.assertEqual((by_name["D0"].side, by_name["D0"].pos), ("L", 1))
        self.assertEqual((by_name["5V"].side, by_name["5V"].pos), ("R", 1))
        self.assertEqual((by_name["D6"].side, by_name["D6"].pos), ("L", 7))
        self.assertEqual((by_name["D7"].side, by_name["D7"].pos), ("R", 7))

    def test_gpio3_is_the_strapping_pin(self):
        self.assertEqual([p.name for p in self.board.pins if p.strapping], ["D2"])


SIDECAR = """\
    [source]
    convention = "robocar"
    header = "main/pin_config.h"
    board = "board.md"

    [parts.led]
    name = "LED"
    kind = "led"

    [[nets]]
    role = "LED_PIN"
    to = "led.A"

    [[undrawn]]
    role = "BEEP_PIN"
    why = "test"
    """


class JoinTest(unittest.TestCase):
    """join() against a synthetic project, so each failure mode is isolated."""

    def make(self, sidecar: str = SIDECAR, header: str = HEADER_H) -> Path:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        write(root / "board.md", BOARD_MD)
        proj = root / "proj"
        write(proj / "main/pin_config.h", header)
        write(proj / "hardware.toml", sidecar)
        return proj

    def join(self, proj: Path):
        return join(proj, repo_root=proj.parent)

    def test_role_resolves_through_the_board_to_a_pad(self):
        model = self.join(self.make())
        self.assertEqual(model.roles, {"LED_PIN": 1, "BEEP_PIN": 7})
        self.assertEqual(model.pin_for("LED_PIN").name, "D0")
        self.assertEqual(model.pin_for("BEEP_PIN").name, "D2")

    def test_net_endpoints_are_split_into_part_and_pin(self):
        (net,) = self.join(self.make()).nets
        self.assertEqual((net.role, net.part, net.pin), ("LED_PIN", "led", "A"))

    def test_extra_headers_supply_defines_but_never_roles(self):
        # A pin role is a fact of `header` alone; a GPIO-shaped macro in an
        # extra header (read only for its constants) must not become one.
        sidecar = SIDECAR.replace(
            'header = "main/pin_config.h"',
            'header = "main/pin_config.h"\nextra_headers = ["main/extra.h"]',
        )
        proj = self.make(sidecar=sidecar)
        write(
            proj / "main/extra.h",
            "#define PERIOD_MS 15000U\n#define STRAY_PIN GPIO_NUM_3\n",
        )
        model = self.join(proj)
        self.assertEqual(model.defines["PERIOD_MS"], "15000U")
        self.assertNotIn("STRAY_PIN", model.roles)

    def test_a_role_on_no_header_pad_resolves_to_none(self):
        header = HEADER_H + "#define MIC_PIN GPIO_NUM_42\n"
        model = self.join(self.make(header=header))
        self.assertIsNone(model.pin_for("MIC_PIN"))

    def test_a_net_role_missing_from_the_header_fails(self):
        sidecar = SIDECAR.replace('role = "LED_PIN"', 'role = "LED_PNI"')
        with self.assertRaisesRegex(HardwareError, "LED_PNI"):
            self.join(self.make(sidecar=sidecar))

    def test_a_net_role_that_is_not_a_pin_fails(self):
        # BUS_FREQ_HZ is a real macro but not a GPIO role.
        sidecar = SIDECAR.replace('role = "LED_PIN"', 'role = "BUS_FREQ_HZ"')
        with self.assertRaisesRegex(HardwareError, "BUS_FREQ_HZ"):
            self.join(self.make(sidecar=sidecar))

    def test_an_undrawn_role_missing_from_the_header_fails(self):
        sidecar = SIDECAR.replace('role = "BEEP_PIN"', 'role = "BEEP_PNI"')
        with self.assertRaisesRegex(HardwareError, "BEEP_PNI"):
            self.join(self.make(sidecar=sidecar))

    def test_a_net_to_an_undeclared_part_fails(self):
        sidecar = SIDECAR.replace('to = "led.A"', 'to = "lde.A"')
        with self.assertRaisesRegex(HardwareError, "lde"):
            self.join(self.make(sidecar=sidecar))

    def test_a_net_endpoint_without_a_pin_fails(self):
        sidecar = SIDECAR.replace('to = "led.A"', 'to = "led"')
        with self.assertRaisesRegex(HardwareError, "part.PIN"):
            self.join(self.make(sidecar=sidecar))

    def test_a_role_both_wired_and_excused_fails(self):
        sidecar = SIDECAR.replace('role = "BEEP_PIN"', 'role = "LED_PIN"')
        with self.assertRaisesRegex(HardwareError, "LED_PIN"):
            self.join(self.make(sidecar=sidecar))

    def test_a_pin_number_in_the_sidecar_is_rejected(self):
        # ADR-021: hardware.toml names roles, never GPIOs. An unknown key is
        # how a pin number would sneak in.
        sidecar = SIDECAR.replace('to = "led.A"', 'to = "led.A"\ngpio = 1')
        with self.assertRaisesRegex(HardwareError, "gpio"):
            self.join(self.make(sidecar=sidecar))

    def test_one_role_on_two_parts_is_a_fan_out(self):
        sidecar = SIDECAR.replace(
            "[[undrawn]]",
            '[parts.scope]\nname = "Probe"\nkind = "probe"\n\n'
            '[[nets]]\nrole = "LED_PIN"\nto = "scope.CH1"\n\n[[undrawn]]',
        )
        nets = self.join(self.make(sidecar=sidecar)).nets
        self.assertEqual([n.part for n in nets], ["led", "scope"])

    def test_the_same_net_twice_fails(self):
        sidecar = SIDECAR.replace(
            "[[undrawn]]", '[[nets]]\nrole = "LED_PIN"\nto = "led.A"\n\n[[undrawn]]'
        )
        with self.assertRaisesRegex(HardwareError, "listed twice"):
            self.join(self.make(sidecar=sidecar))

    def test_the_same_role_excused_twice_fails(self):
        sidecar = SIDECAR + '\n[[undrawn]]\nrole = "BEEP_PIN"\nwhy = "again"\n'
        with self.assertRaisesRegex(HardwareError, "excused twice"):
            self.join(self.make(sidecar=sidecar))

    def test_a_part_note_is_kept(self):
        sidecar = SIDECAR.replace('kind = "led"', 'kind = "led"\nnote = "red"')
        self.assertEqual(self.join(self.make(sidecar=sidecar)).parts["led"].note, "red")

    def test_wrongly_shaped_values_fail_as_hardware_errors(self):
        # A traceback would bypass the generator's one-line `error:` exit.
        for broken in (
            SIDECAR.replace(
                'header = "main/pin_config.h"',
                'header = "main/pin_config.h"\nextra_headers = "main/x.h"',
            ),
            SIDECAR.replace("[[nets]]", "[nets.x]"),
            SIDECAR.replace("[parts.led]", "[parts]\nled = 5\n[parts.other]"),
            "source = 5\n",
        ):
            with self.subTest(broken=broken), self.assertRaises(HardwareError):
                self.join(self.make(sidecar=broken))

    def test_missing_sidecar_fails(self):
        proj = self.make()
        (proj / "hardware.toml").unlink()
        with self.assertRaisesRegex(HardwareError, "hardware.toml"):
            self.join(proj)


class RobocarUnifiedJoinTest(unittest.TestCase):
    """The committed sidecar resolves against the committed header and board."""

    @classmethod
    def setUpClass(cls):
        cls.model = join(UNIFIED)

    def test_i2s_lands_on_the_amp_via_d8_d9_d10(self):
        nets = {n.role: (n.part, n.pin) for n in self.model.nets}
        self.assertEqual(nets["I2S_BCLK_PIN"], ("amp", "BCLK"))
        self.assertEqual(self.model.pin_for("I2S_BCLK_PIN").name, "D8")
        self.assertEqual(self.model.pin_for("I2S_LRCLK_PIN").name, "D9")
        self.assertEqual(self.model.pin_for("I2S_DIN_PIN").name, "D10")

    def test_the_microphone_is_excused_and_on_no_header_pad(self):
        undrawn = {u.role for u in self.model.undrawn}
        self.assertEqual(undrawn, {"MIC_PDM_CLK_PIN", "MIC_PDM_DATA_PIN"})
        for role in undrawn:
            self.assertIsNone(self.model.pin_for(role), role)

    def test_headers_are_pin_config_then_the_planner_headers(self):
        rel = [p.relative_to(UNIFIED).as_posix() for p in self.model.headers]
        self.assertEqual(
            rel, ["main/pin_config.h", "main/planner_task.h", "main/plan_activity.h"]
        )


class GeneratorTest(unittest.TestCase):
    """Issue #459's acceptance: pin_defs.typ comes out byte-identical."""

    committed = UNIFIED / "docs/auto/pin_defs.typ"

    def run_generator(self, cwd: Path, project: str) -> bytes:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "pin_defs.typ"
            subprocess.run(
                [sys.executable, str(GENERATOR), project, str(out)],
                cwd=cwd,
                check=True,
                capture_output=True,
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            )
            return out.read_bytes()

    def test_output_from_repo_root_matches_the_committed_file(self):
        out = self.run_generator(REPO_ROOT, "packages/robocar/unified")
        self.assertEqual(out, self.committed.read_bytes())

    def test_output_from_the_project_directory_is_identical(self):
        # The justfile runs it from the project, CI from the repo root.
        out = self.run_generator(UNIFIED, ".")
        self.assertEqual(out, self.committed.read_bytes())


if __name__ == "__main__":
    unittest.main()
