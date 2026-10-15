"""Tests for tools/hardware — the board × header × parts join (ADR-021, #459).

Stdlib `unittest` only, so the suite runs wherever the generator does (CI's
system python3, the justfile, the pre-commit hook) with nothing to install:

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import importlib.util
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest import mock

import hardware.header
import hardware.layout
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
BALANCEBOT = REPO_ROOT / "packages/robotics/balancebot"
XIAO_RP2350 = REPO_ROOT / "docs/reference/boards/xiao-rp2350.md"
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
        with self.assertRaisesRegex(HardwareError, "balancebot.*robocar"):
            roles_from_defines({}, "gamepad-synth")

    def test_a_bare_integer_is_not_a_robocar_pin_even_with_a_pin_name(self):
        self.assertEqual(roles_from_defines({"PIN_IMU_SDA": "6"}, "robocar"), {})


class BalancebotConventionTest(unittest.TestCase):
    def test_pin_prefixed_bare_integers_are_roles(self):
        defines = {
            "PIN_IMU_SDA": "6",
            "PIN_STATUS_LED": "25",
            "CONTROL_LOOP_HZ": "500",  # a bare integer, but not a pin
            "MPU6050_ADDR": "0x68",
            "SPIN_COUNT": "3",  # PIN_ inside the name, not at its start
            "PIN_ALIAS": "PIN_IMU_SDA",  # not an integer
        }
        self.assertEqual(
            roles_from_defines(defines, "balancebot"),
            {"PIN_IMU_SDA": 6, "PIN_STATUS_LED": 25},
        )

    def test_gpio_num_values_are_not_balancebot_pins(self):
        self.assertEqual(roles_from_defines({"PIN_X": "GPIO_NUM_5"}, "balancebot"), {})

    def test_the_real_header(self):
        # Every #define in src/pin_config.h except the include guard is a pin.
        defines = parse_defines([BALANCEBOT / "src/pin_config.h"])
        self.assertEqual(
            roles_from_defines(defines, "balancebot"),
            {
                "PIN_IMU_SDA": 6,
                "PIN_IMU_SCL": 7,
                "PIN_IMU_INT": 5,
                "PIN_STEP_LEFT": 2,
                "PIN_STEP_RIGHT": 3,
                "PIN_DIR_LEFT": 4,
                "PIN_DIR_RIGHT": 0,
                "PIN_MOTOR_NENABLE": 1,
                "PIN_STATUS_LED": 25,
                "PIN_BATT_ADC": 29,
                "PIN_BATT_ADC_ENABLE": 19,
                "PIN_PACK_SENSE": 26,
            },
        )


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


class XiaoRp2350ReferenceTest(unittest.TestCase):
    """The board reference a balancebot join reads."""

    @classmethod
    def setUpClass(cls):
        cls.board = parse_board_table(XIAO_RP2350)

    def test_all_fourteen_header_positions_are_listed(self):
        # Exactly the side header: the bottom-pad table must not be read too.
        self.assertEqual(len(self.board.pins), 14)
        self.assertNotIn("D11", self.board.by_name)

    def test_each_side_is_positions_one_to_seven(self):
        for side in ("L", "R"):
            positions = sorted(p.pos for p in self.board.pins if p.side == side)
            self.assertEqual(positions, list(range(1, 8)), side)

    def test_physical_order_matches_seeeds_footprint(self):
        # Seeed's XIAO-RP2350-DIP footprint: pads 1-7 (D0-D6) down one edge,
        # 8-14 (D7, D8, D9, D10, 3V3_OUT, GND, VBUS) back up the other, pads 1
        # and 14 beside the USB-C connector.
        expect = {
            "D0": ("L", 1), "D6": ("L", 7),
            "5V": ("R", 1), "GND": ("R", 2), "3V3": ("R", 3),
            "D10": ("R", 4), "D9": ("R", 5), "D8": ("R", 6), "D7": ("R", 7),
        }  # fmt: skip
        for name, where in expect.items():
            pin = self.board.by_name[name]
            self.assertEqual((pin.side, pin.pos), where, name)

    def test_silkscreen_to_gpio_agrees_with_balancebots_header_comments(self):
        # src/pin_config.h names the XIAO pad beside each GPIO.
        expect = {
            "D3": 5,
            "D4": 6,
            "D5": 7,
            "D6": 0,
            "D7": 1,
            "D8": 2,
            "D9": 4,
            "D10": 3,
        }
        for name, gpio in expect.items():
            self.assertEqual(self.board.by_name[name].gpio, gpio, name)

    def test_power_pads_have_no_gpio(self):
        for name in ("5V", "GND", "3V3"):
            self.assertIsNone(self.board.by_name[name].gpio, name)

    def test_layout_reads_the_same_table(self):
        layout = hardware.layout.parse_layout(XIAO_RP2350)
        self.assertEqual(
            [p.name for p in layout.side("R")],
            ["5V", "GND", "3V3", "D10", "D9", "D8", "D7"],
        )


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
        sidecar = SIDECAR + '\n[[undrawn]]\nrole = "MIC_PIN"\nwhy = "test"\n'
        model = self.join(self.make(sidecar=sidecar, header=header))
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

    def test_a_pin_role_on_no_net_and_not_undrawn_fails(self):
        # The completeness gate (#460): a new pin macro must be drawn or excused.
        header = HEADER_H + "#define LIDAR_PWM_PIN GPIO_NUM_12\n"
        with self.assertRaises(HardwareError) as ctx:
            self.join(self.make(header=header))
        self.assertIn("LIDAR_PWM_PIN", str(ctx.exception))
        self.assertIn("GPIO12", str(ctx.exception))
        self.assertIn("[[nets]]", str(ctx.exception))
        self.assertIn("[[undrawn]]", str(ctx.exception))

    def test_an_excused_unwired_pin_role_joins(self):
        header = HEADER_H + "#define LIDAR_PWM_PIN GPIO_NUM_12\n"
        sidecar = SIDECAR + '\n[[undrawn]]\nrole = "LIDAR_PWM_PIN"\nwhy = "test"\n'
        model = self.join(self.make(sidecar=sidecar, header=header))
        self.assertEqual(model.roles["LIDAR_PWM_PIN"], 12)

    def test_a_non_pin_macro_is_not_flagged_by_the_completeness_gate(self):
        header = HEADER_H + "#define LIDAR_FREQ_HZ 50\n"
        self.join(self.make(header=header))

    def test_every_missing_role_is_named_not_just_the_first(self):
        header = HEADER_H + "#define A_PIN GPIO_NUM_12\n#define B_PIN GPIO_NUM_13\n"
        with self.assertRaises(HardwareError) as ctx:
            self.join(self.make(header=header))
        self.assertIn("A_PIN", str(ctx.exception))
        self.assertIn("B_PIN", str(ctx.exception))

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

    def test_two_roles_on_one_gpio_fail_naming_both_and_the_gpio(self):
        # #722: a pin conflict is a wiring error, so the join reports it
        # rather than leaving it to surface as a router failure. The second
        # role is excused, so the conflict alone can be what fails.
        header = HEADER_H + "#define PIEZO_PIN GPIO_NUM_1\n"
        sidecar = SIDECAR + '\n[[undrawn]]\nrole = "PIEZO_PIN"\nwhy = "test"\n'
        with self.assertRaises(HardwareError) as ctx:
            self.join(self.make(sidecar=sidecar, header=header))
        message = str(ctx.exception)
        self.assertIn("LED_PIN", message)
        self.assertIn("PIEZO_PIN", message)
        self.assertIn("GPIO1", message)

    def test_a_macro_naming_another_role_is_an_alias_not_a_second_role(self):
        # The allowance for a pin meant to share: write it as the other
        # role's name, which no convention reads as a second GPIO.
        header = HEADER_H + "#define STATUS_PIN LED_PIN\n"
        model = self.join(self.make(header=header))
        self.assertNotIn("STATUS_PIN", model.roles)

    def test_two_roles_onto_one_pad_of_a_part_fail(self):
        # Two GPIOs driving one pin is a short between them; one role on two
        # pins (a fan-out) stays allowed.
        sidecar = SIDECAR.replace("[[undrawn]]", "[[nets]]").replace(
            'why = "test"', 'to = "led.A"'
        )
        self.assertEqual(sidecar.count("[[nets]]"), 2)
        with self.assertRaises(HardwareError) as ctx:
            self.join(self.make(sidecar=sidecar))
        message = str(ctx.exception)
        self.assertIn("led.A", message)
        self.assertIn("LED_PIN", message)
        self.assertIn("BEEP_PIN", message)

    def test_the_same_role_excused_twice_fails(self):
        sidecar = SIDECAR + '\n[[undrawn]]\nrole = "BEEP_PIN"\nwhy = "again"\n'
        with self.assertRaisesRegex(HardwareError, "excused twice"):
            self.join(self.make(sidecar=sidecar))

    def test_a_part_note_is_kept(self):
        sidecar = SIDECAR.replace('kind = "led"', 'kind = "led"\nnote = "red"')
        self.assertEqual(self.join(self.make(sidecar=sidecar)).parts["led"].note, "red")

    def test_a_part_board_is_kept_and_defaults_to_none(self):
        self.assertEqual(self.join(self.make()).parts["led"].board, "")
        sidecar = SIDECAR.replace('kind = "led"', 'kind = "led"\nboard = "board.md"')
        self.assertEqual(
            self.join(self.make(sidecar=sidecar)).parts["led"].board, "board.md"
        )

    def test_a_sidecar_without_carrier_keys_joins_with_them_empty(self):
        # The #750 keys are all optional: robocar's sidecar has none of them.
        model = self.join(self.make())
        led = model.parts["led"]
        self.assertEqual((led.part, led.ref, led.value, led.lcsc, led.build), ("",) * 5)
        self.assertEqual((model.view, model.net_names), ("harness", {}))
        self.assertEqual((model.mcu_part, model.mcu_ref), ("", ""))

    def test_a_part_board_that_does_not_exist_fails(self):
        sidecar = SIDECAR.replace('kind = "led"', 'kind = "led"\nboard = "nope.md"')
        with self.assertRaisesRegex(
            HardwareError, r"\[parts.led\].*nope.md does not exist"
        ):
            self.join(self.make(sidecar=sidecar))

    def test_a_part_board_that_is_not_a_path_fails(self):
        for value in ('""', "5"):
            sidecar = SIDECAR.replace('kind = "led"', f'kind = "led"\nboard = {value}')
            with (
                self.subTest(value=value),
                self.assertRaisesRegex(HardwareError, "board"),
            ):
                self.join(self.make(sidecar=sidecar))

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


POWER = """
[parts.pack]
name = "Pack"
kind = "battery"

[parts.lamp]
name = "Lamp"
kind = "lamp"

[[rails]]
name = "5V"
from = "pack.+"
to = ["mcu.5V", "led.VCC"]

[[outputs]]
from = "led"
to = ["lamp"]
"""


class PowerTest(unittest.TestCase):
    """[[rails]] and [[outputs]] (#646): the supply side of the join."""

    make = JoinTest.make
    join = JoinTest.join

    def power(self, edit=lambda s: s):
        return self.join(self.make(sidecar=edit(SIDECAR + POWER)))

    def test_a_rail_is_split_into_its_source_and_loads(self):
        (rail,) = self.power().rails
        self.assertEqual(rail.name, "5V")
        self.assertEqual((rail.source.part, rail.source.pin), ("pack", "+"))
        self.assertEqual(
            [(e.part, e.pin) for e in rail.loads], [("mcu", "5V"), ("led", "VCC")]
        )

    def test_an_output_names_the_loads_a_part_drives(self):
        (out,) = self.power().outputs
        self.assertEqual((out.part, out.loads), ("led", ("lamp",)))

    def test_the_mcu_name_defaults_and_can_be_set(self):
        self.assertEqual(self.power().mcu, "MCU")
        model = self.power(
            lambda s: s.replace('board = "board.md"', 'board = "board.md"\nmcu = "Dev"')
        )
        self.assertEqual(model.mcu, "Dev")

    def test_an_mcu_endpoint_must_be_a_pad_on_the_board(self):
        # The test board has a 5V pad and no 3V3 pad.
        with self.assertRaisesRegex(HardwareError, "mcu.3V3.*no pad named '3V3'"):
            self.power(lambda s: s.replace('"mcu.5V"', '"mcu.3V3"'))

    def test_a_rail_pin_must_be_on_the_parts_board_where_it_has_one(self):
        # led's board is the test board: 5V is a pad there, VCC is not.
        def edit(s):
            return s.replace('kind = "led"', 'kind = "led"\nboard = "board.md"')

        with self.assertRaisesRegex(HardwareError, "led.VCC.*no pad named 'VCC'"):
            self.power(edit)
        model = self.power(lambda s: edit(s).replace('"led.VCC"', '"led.5V"'))
        self.assertEqual(model.rails[0].loads[1].pin, "5V")

    def test_a_rail_to_an_undeclared_part_fails(self):
        with self.assertRaisesRegex(HardwareError, "part 'lde' is not declared"):
            self.power(lambda s: s.replace('"led.VCC"', '"lde.VCC"'))

    def test_a_rail_endpoint_without_a_pin_fails(self):
        with self.assertRaisesRegex(HardwareError, "part.PIN"):
            self.power(lambda s: s.replace('from = "pack.+"', 'from = "pack"'))

    def test_a_rail_with_no_loads_fails(self):
        with self.assertRaisesRegex(HardwareError, "'to' must be a non-empty list"):
            self.power(lambda s: s.replace('to = ["mcu.5V", "led.VCC"]', "to = []"))

    def test_one_pin_on_two_rails_is_a_short(self):
        extra = '\n[[rails]]\nname = "3V3"\nfrom = "pack.-"\nto = ["led.VCC"]\n'
        with self.assertRaisesRegex(
            HardwareError, "led.VCC is on rail '5V' and rail '3V3'"
        ):
            self.power(lambda s: s + extra)

    def test_a_pin_listed_twice_on_one_rail_is_a_duplicate_not_a_short(self):
        with self.assertRaisesRegex(HardwareError, "led.VCC.*listed twice"):
            self.power(lambda s: s.replace('"led.VCC"]', '"led.VCC", "led.VCC"]'))

    def test_a_rail_pin_that_is_also_a_signal_net_fails(self):
        # led.A carries LED_PIN; putting it on a supply shorts a GPIO to a rail.
        with self.assertRaisesRegex(HardwareError, "led.A.*LED_PIN"):
            self.power(lambda s: s.replace('"led.VCC"', '"led.A"'))

    def test_an_mcu_pad_that_carries_a_signal_net_fails(self):
        # D0 is GPIO1, which LED_PIN drives: the MCU end of the same short.
        with self.assertRaisesRegex(HardwareError, "mcu.D0.*LED_PIN"):
            self.power(lambda s: s.replace('"mcu.5V"', '"mcu.D0"'))

    def test_a_part_may_not_take_the_mcu_id(self):
        with self.assertRaisesRegex(HardwareError, "'mcu' is reserved"):
            self.power(lambda s: s.replace("[parts.lamp]", "[parts.mcu]"))

    def test_an_output_to_an_undeclared_part_fails(self):
        with self.assertRaisesRegex(HardwareError, "part 'lmap' is not declared"):
            self.power(lambda s: s.replace('to = ["lamp"]', 'to = ["lmap"]'))

    def test_an_output_listing_a_load_twice_fails(self):
        with self.assertRaisesRegex(HardwareError, "lamp.*twice"):
            self.power(lambda s: s.replace('to = ["lamp"]', 'to = ["lamp", "lamp"]'))

    def test_unknown_rail_keys_are_rejected(self):
        # A voltage number is prose; the rail's name is its only voltage fact.
        with self.assertRaisesRegex(HardwareError, "volts"):
            self.power(lambda s: s.replace('name = "5V"', 'name = "5V"\nvolts = 5'))


DRIVER_MD = """\
    # Test PWM driver

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VCC | L | 1 | JP1.1 | |
    | 0 | B | 1 | JP2.1 | |
    | 1 | B | 2 | JP2.2 | |
    | 2 | B | 3 | JP2.3 | |
    """

MOTOR_MD = """\
    # Test motor driver

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | PWMA | L | 1 | J1.1 | |
    | AIN1 | L | 2 | J1.2 | |
    | STBY | L | 3 | J1.3 | |
    | VM | R | 1 | J2.1 | |
    """

CHANNEL_HEADER = (
    HEADER_H
    + """\
    #define I2C_BUS_CHANNEL_DRV 0
    #define BLOCK_FIRST_CHANNEL 1
    #define SPEED_CHANNEL 1
    #define DIR_CHANNEL 2
    #define LAMP_CHANNEL 0
    """
)

CHANNELS = """
[parts.drv]
name = "PWM chip"
kind = "pwm-driver"
board = "driver.md"

[parts.motor]
name = "H-bridge"
kind = "motor-driver"
board = "motor.md"

[parts.lamp]
name = "Lamp"
kind = "lamp"

[[channel_nets]]
role = "SPEED_CHANNEL"
from = "drv"
to = "motor.PWMA"
note = "speed"

[[channel_nets]]
role = "DIR_CHANNEL"
from = "drv"
to = "motor.AIN1"

[[channel_nets]]
role = "LAMP_CHANNEL"
from = "drv"
to = "lamp.K"
"""


class ChannelNetTest(unittest.TestCase):
    """[[channel_nets]] (#666): nets that start at a PWM driver's output."""

    def make(self, sidecar: str, header: str = CHANNEL_HEADER) -> Path:
        proj = JoinTest.make(self, sidecar=sidecar, header=header)
        write(proj.parent / "driver.md", DRIVER_MD)
        write(proj.parent / "motor.md", MOTOR_MD)
        return proj

    join = JoinTest.join

    def channels(self, edit=lambda s: s, header: str = CHANNEL_HEADER):
        return self.join(self.make(edit(SIDECAR + CHANNELS), header=header))

    def test_channel_roles_come_from_the_header_by_name_and_value(self):
        model = self.channels()
        self.assertEqual(
            model.channels,
            {
                "BLOCK_FIRST_CHANNEL": 1,
                "SPEED_CHANNEL": 1,
                "DIR_CHANNEL": 2,
                "LAMP_CHANNEL": 0,
            },
        )
        # A multiplexer channel is not a driver output, and a pin is not one.
        self.assertNotIn("I2C_BUS_CHANNEL_DRV", model.channels)
        self.assertNotIn("LED_PIN", model.channels)

    def test_a_channel_net_resolves_to_its_number_and_both_ends(self):
        net = self.channels().channel_nets[0]
        self.assertEqual(
            (net.role, net.channel, net.source, net.part, net.pin, net.note),
            ("SPEED_CHANNEL", 1, "drv", "motor", "PWMA", "speed"),
        )

    def test_channel_nets_leave_the_mcu_nets_alone(self):
        self.assertEqual([n.role for n in self.channels().nets], ["LED_PIN"])

    def test_a_pin_role_is_not_a_channel_role(self):
        with self.assertRaisesRegex(HardwareError, "LED_PIN.*not a channel role"):
            self.channels(lambda s: s.replace('"SPEED_CHANNEL"', '"LED_PIN"'))

    def test_an_undefined_channel_role_fails(self):
        with self.assertRaisesRegex(HardwareError, "SPEED_CHANNLE.*not defined"):
            self.channels(lambda s: s.replace('"SPEED_CHANNEL"', '"SPEED_CHANNLE"'))

    def test_a_channel_number_in_the_sidecar_is_rejected(self):
        # ADR-021 again: the number lives in the header, never here.
        with self.assertRaisesRegex(HardwareError, "channel"):
            self.channels(lambda s: s.replace('note = "speed"', "channel = 1"))

    def test_the_channel_must_be_a_pad_on_the_drivers_board(self):
        header = CHANNEL_HEADER.replace("DIR_CHANNEL 2", "DIR_CHANNEL 7")
        with self.assertRaisesRegex(
            HardwareError, "DIR_CHANNEL.*channel 7.*driver.md has no pad named '7'"
        ):
            self.channels(header=header)

    def test_the_far_pin_must_be_a_pad_on_its_board(self):
        with self.assertRaisesRegex(HardwareError, "motor.BIN1.*no pad named 'BIN1'"):
            self.channels(lambda s: s.replace('"motor.AIN1"', '"motor.BIN1"'))

    def test_two_wired_roles_on_one_channel_fail(self):
        # The header and hardware.toml disagree: pin_config.h has put the
        # direction line on the speed line's channel.
        header = CHANNEL_HEADER.replace("DIR_CHANNEL 2", "DIR_CHANNEL 1")
        with self.assertRaisesRegex(
            HardwareError, "channel 1 of drv.*SPEED_CHANNEL.*DIR_CHANNEL"
        ):
            self.channels(header=header)

    def test_an_unwired_alias_of_a_wired_channel_is_fine(self):
        # BLOCK_FIRST_CHANNEL names the same channel as SPEED_CHANNEL; only one
        # of them is wired, so nothing is driven twice.
        self.assertEqual(len(self.channels().channel_nets), 3)

    def test_one_channel_to_two_pins_is_a_fan_out(self):
        extra = (
            '\n[[channel_nets]]\nrole = "LAMP_CHANNEL"\nfrom = "drv"\nto = "lamp.K2"\n'
        )
        nets = self.channels(lambda s: s + extra).channel_nets
        self.assertEqual([n.pin for n in nets if n.role == "LAMP_CHANNEL"], ["K", "K2"])

    def test_the_same_channel_net_twice_fails(self):
        extra = (
            '\n[[channel_nets]]\nrole = "LAMP_CHANNEL"\nfrom = "drv"\nto = "lamp.K"\n'
        )
        with self.assertRaisesRegex(HardwareError, "listed twice"):
            self.channels(lambda s: s + extra)

    def test_a_pin_driven_by_two_channels_fails(self):
        with self.assertRaisesRegex(
            HardwareError, "motor.PWMA.*SPEED_CHANNEL.*DIR_CHANNEL"
        ):
            self.channels(lambda s: s.replace('"motor.AIN1"', '"motor.PWMA"'))

    def test_a_pin_driven_by_the_mcu_and_a_channel_fails(self):
        # BEEP_PIN moves from [[undrawn]] onto a net to motor.STBY, and a
        # channel then claims the same pin.
        def edit(s: str) -> str:
            excused = '[[undrawn]]\n    role = "BEEP_PIN"\n    why = "test"'
            return (
                s.replace(excused, "")
                + '\n[[nets]]\nrole = "BEEP_PIN"\nto = "motor.STBY"\n'
                + '\n[[channel_nets]]\nrole = "LAMP_CHANNEL"\n'
                + 'from = "drv"\nto = "motor.STBY"\n'
            )

        with self.assertRaisesRegex(
            HardwareError, "motor.STBY.*BEEP_PIN.*LAMP_CHANNEL"
        ):
            self.channels(edit)

    def test_an_mcu_net_onto_a_channel_pad_fails(self):
        # The driver's channel pad is an output: a GPIO wired to it is a second
        # driver on the same node, at the source end rather than the far one.
        def edit(s: str) -> str:
            excused = '[[undrawn]]\n    role = "BEEP_PIN"\n    why = "test"'
            return (
                s.replace(excused, "") + '\n[[nets]]\nrole = "BEEP_PIN"\nto = "drv.1"\n'
            )

        with self.assertRaisesRegex(HardwareError, "drv.1.*BEEP_PIN.*SPEED_CHANNEL"):
            self.channels(edit)

    def test_a_sidecar_without_channel_nets_needs_no_channel_convention(self):
        # A project adopting hardware.toml extends CONVENTIONS; until it wires a
        # PWM driver it must not also need a CHANNEL_CONVENTIONS entry.
        with mock.patch.dict(hardware.header.CHANNEL_CONVENTIONS, clear=True):
            model = self.join(self.make(SIDECAR))
            self.assertEqual((model.channels, model.channel_nets), ({}, ()))
            with self.assertRaisesRegex(HardwareError, "no channel convention"):
                self.channels()

    def test_a_channel_net_from_an_undeclared_part_fails(self):
        with self.assertRaisesRegex(HardwareError, "part 'dvr' is not declared"):
            self.channels(lambda s: s.replace('from = "drv"', 'from = "dvr"', 1))

    def test_a_channel_net_to_its_own_driver_fails(self):
        with self.assertRaisesRegex(HardwareError, "drv.*its own"):
            self.channels(lambda s: s.replace('"lamp.K"', '"drv.VCC"'))

    def test_a_rail_on_a_channel_net_fails(self):
        rail = '\n[[rails]]\nname = "5V"\nfrom = "mcu.5V"\nto = ["{}"]\n'
        for pin, role in (("motor.PWMA", "SPEED_CHANNEL"), ("drv.1", "SPEED_CHANNEL")):
            with (
                self.subTest(pin=pin),
                self.assertRaisesRegex(HardwareError, f"{pin}.*{role}"),
            ):
                self.channels(lambda s, p=pin: s + rail.format(p))


MUX_MD = """\
    # Test I2C multiplexer

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VIN | L | 1 | J1.1 | |
    | SDA | L | 2 | J1.2 | Upstream |
    | SCL | L | 3 | J1.3 | Upstream |
    | SD0 | R | 1 | J2.1 | |
    | SC0 | R | 2 | J2.2 | |
    | SD1 | R | 3 | J2.3 | |
    | SC1 | R | 4 | J2.4 | |
    """

SCREEN_MD = """\
    # Test I2C screen

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VCC | L | 1 | J1.1 | |
    | SDA | L | 2 | J1.2 | |
    | SCL | L | 3 | J1.3 | |
    """

BUS_HEADER = (
    HEADER_H
    + """\
    #define I2C_BUS_CHANNEL_SCREEN 0
    #define I2C_BUS_CHANNEL_PROBE 1
    #define SCREEN_ADDR 0x3C
    #define LAMP_CHANNEL 0
    """
)

BUSES = """
[parts.mux]
name = "I2C mux"
kind = "i2c-mux"
board = "mux.md"

[parts.screen]
name = "Screen"
kind = "display"
board = "screen.md"

[parts.probe]
name = "Probe"
kind = "sensor"

[[bus_nets]]
role = "I2C_BUS_CHANNEL_SCREEN"
from = "mux"
to = "screen"
note = "display"

[[bus_nets]]
role = "I2C_BUS_CHANNEL_PROBE"
from = "mux"
to = "probe"
"""

EXCUSED = '[[undrawn]]\n    role = "BEEP_PIN"\n    why = "test"'


class BusNetTest(unittest.TestCase):
    """[[bus_nets]] (#768): an I2C mux channel's SDn/SCn pair to a load's SDA/SCL."""

    def make(self, sidecar: str, header: str = BUS_HEADER) -> Path:
        proj = JoinTest.make(self, sidecar=sidecar, header=header)
        write(proj.parent / "mux.md", MUX_MD)
        write(proj.parent / "screen.md", SCREEN_MD)
        return proj

    join = JoinTest.join

    def buses(self, edit=lambda s: s, header: str = BUS_HEADER, view: str = "harness"):
        proj = self.make(edit(SIDECAR + BUSES), header=header)
        return join(proj, repo_root=proj.parent, view=view)

    def test_bus_channel_roles_come_from_the_header_by_name_and_value(self):
        model = self.buses()
        self.assertEqual(
            model.bus_channels,
            {"I2C_BUS_CHANNEL_SCREEN": 0, "I2C_BUS_CHANNEL_PROBE": 1},
        )
        # A PWM-driver channel and an address are not mux channels.
        self.assertNotIn("LAMP_CHANNEL", model.bus_channels)
        self.assertNotIn("SCREEN_ADDR", model.bus_channels)

    def test_a_bus_net_resolves_to_its_channel_and_both_pads_of_each_line(self):
        screen, probe = self.buses().bus_nets
        self.assertEqual(
            (screen.role, screen.channel, screen.source, screen.part, screen.note),
            ("I2C_BUS_CHANNEL_SCREEN", 0, "mux", "screen", "display"),
        )
        self.assertEqual(screen.wires, (("SD0", "SDA"), ("SC0", "SCL")))
        self.assertEqual(probe.wires, (("SD1", "SDA"), ("SC1", "SCL")))

    def test_bus_nets_leave_the_mcu_and_channel_nets_alone(self):
        model = self.buses()
        self.assertEqual([n.role for n in model.nets], ["LED_PIN"])
        self.assertEqual(model.channel_nets, ())

    def test_a_pin_role_is_not_a_bus_channel_role(self):
        with self.assertRaisesRegex(HardwareError, "LED_PIN.*not a bus channel role"):
            self.buses(lambda s: s.replace('"I2C_BUS_CHANNEL_PROBE"', '"LED_PIN"'))

    def test_an_undefined_bus_channel_role_fails(self):
        typo = '"I2C_BUS_CHANNEL_PRBOE"'
        with self.assertRaisesRegex(HardwareError, "PRBOE.*not defined"):
            self.buses(lambda s: s.replace('"I2C_BUS_CHANNEL_PROBE"', typo))

    def test_a_channel_number_in_the_sidecar_is_rejected(self):
        with self.assertRaisesRegex(HardwareError, "channel"):
            self.buses(lambda s: s.replace('note = "display"', "channel = 0"))

    def test_the_channel_pair_must_be_pads_on_the_muxs_board(self):
        header = BUS_HEADER.replace("CHANNEL_PROBE 1", "CHANNEL_PROBE 5")
        with self.assertRaisesRegex(
            HardwareError, "PROBE is channel 5.*mux.md has no pad named 'SD5'"
        ):
            self.buses(header=header)

    def test_the_load_needs_sda_and_scl_on_its_board(self):
        proj = self.make(SIDECAR + BUSES)
        write(
            proj.parent / "screen.md",
            SCREEN_MD.replace("    | SCL | L | 3 | J1.3 | |\n", ""),
        )
        with self.assertRaisesRegex(HardwareError, "screen.SCL.*no pad named 'SCL'"):
            self.join(proj)

    def test_two_wired_roles_on_one_mux_channel_fail(self):
        header = BUS_HEADER.replace("CHANNEL_PROBE 1", "CHANNEL_PROBE 0")
        with self.assertRaisesRegex(
            HardwareError, "channel 0 of mux.*CHANNEL_SCREEN.*CHANNEL_PROBE"
        ):
            self.buses(header=header)

    def test_one_channel_to_two_loads_is_a_fan_out(self):
        extra = (
            '\n[[bus_nets]]\nrole = "I2C_BUS_CHANNEL_PROBE"\nfrom = "mux"\nto = "led"\n'
        )
        buses = self.buses(lambda s: s + extra).bus_nets
        self.assertEqual(
            [b.part for b in buses if b.role == "I2C_BUS_CHANNEL_PROBE"],
            ["probe", "led"],
        )

    def test_the_same_bus_net_twice_fails(self):
        extra = '\n[[bus_nets]]\nrole = "I2C_BUS_CHANNEL_PROBE"\nfrom = "mux"\nto = "probe"\n'
        with self.assertRaisesRegex(HardwareError, "listed twice"):
            self.buses(lambda s: s + extra)

    def test_a_load_on_two_channels_fails(self):
        # Its SDA would join channel 0 to channel 1, which the mux isolates.
        with self.assertRaisesRegex(
            HardwareError, "screen.SDA.*CHANNEL_SCREEN.*CHANNEL_PROBE"
        ):
            self.buses(lambda s: s.replace('to = "probe"', 'to = "screen"'))

    def test_a_load_on_a_bus_and_an_mcu_net_fails(self):
        def edit(s: str) -> str:
            net = '\n[[nets]]\nrole = "BEEP_PIN"\nto = "screen.SDA"\n'
            return s.replace(EXCUSED, "") + net

        with self.assertRaisesRegex(HardwareError, "screen.SDA.*BEEP_PIN.*SCREEN"):
            self.buses(edit)

    def test_an_mcu_net_onto_a_channel_pad_fails(self):
        def edit(s: str) -> str:
            net = '\n[[nets]]\nrole = "BEEP_PIN"\nto = "mux.SC1"\n'
            return s.replace(EXCUSED, "") + net

        with self.assertRaisesRegex(HardwareError, "mux.SC1.*BEEP_PIN.*PROBE"):
            self.buses(edit)

    def test_a_rail_on_a_bus_pin_fails(self):
        rail = '\n[[rails]]\nname = "3V3"\nfrom = "mcu.5V"\nto = ["{}"]\n'
        for pin, role in (
            ("screen.SCL", "I2C_BUS_CHANNEL_SCREEN"),
            ("mux.SD1", "I2C_BUS_CHANNEL_PROBE"),
        ):
            with (
                self.subTest(pin=pin),
                self.assertRaisesRegex(HardwareError, f"{pin}.*{role}"),
            ):
                self.buses(lambda s, p=pin: s + rail.format(p))

    def test_a_bus_net_to_its_own_mux_fails(self):
        with self.assertRaisesRegex(HardwareError, "mux.*its own"):
            self.buses(lambda s: s.replace('to = "probe"', 'to = "mux"'))

    def test_a_bus_net_to_an_undeclared_part_fails(self):
        with self.assertRaisesRegex(HardwareError, "part 'prbe' is not declared"):
            self.buses(lambda s: s.replace('to = "probe"', 'to = "prbe"'))

    def test_to_is_one_part_id_not_a_pin(self):
        with self.assertRaisesRegex(HardwareError, "'to' must be a part id"):
            self.buses(lambda s: s.replace('to = "probe"', 'to = "probe.SDA"'))

    def test_a_sidecar_without_bus_nets_needs_no_bus_convention(self):
        with mock.patch.dict(hardware.header.BUS_CONVENTIONS, clear=True):
            model = self.join(self.make(SIDECAR))
            self.assertEqual((model.bus_channels, model.bus_nets), ({}, ()))
            with self.assertRaisesRegex(HardwareError, "no bus-channel convention"):
                self.buses()

    def test_a_bus_role_is_a_carrier_net_names_key(self):
        names = '\n[net_names]\nI2C_BUS_CHANNEL_SCREEN = "SCREEN"\n'
        model = self.buses(lambda s: s + names, view="carrier")
        self.assertEqual(model.net_names, {"I2C_BUS_CHANNEL_SCREEN": "SCREEN"})

    def test_a_carrier_net_name_cannot_collide_with_a_bus_line(self):
        # The bus's SDA line is <name>_SDA; another net renamed onto it would
        # join two nets through the name alone.
        names = (
            '\n[net_names]\nI2C_BUS_CHANNEL_SCREEN = "SCREEN"\nLED_PIN = "SCREEN_SDA"\n'
        )
        with self.assertRaisesRegex(
            HardwareError, "'SCREEN_SDA'.*role LED_PIN.*I2C_BUS_CHANNEL_SCREEN SDA"
        ):
            self.buses(lambda s: s + names, view="carrier")


CABLE = """
[[cables]]
part = "test-cable"
head_ref = "J1"
main_ref = "J2"
head_no_connect = ["D1"]
build = "carrier"
"""


class CableTest(unittest.TestCase):
    """[[cables]] (#750): one carrier, two boards joined by a cable."""

    make = JoinTest.make

    def cables(self, edit=lambda s: s, view: str = "carrier"):
        proj = self.make(sidecar=edit(SIDECAR + CABLE))
        return join(proj, repo_root=proj.parent, view=view)

    def test_the_carrier_view_reads_the_cable(self):
        (cable,) = self.cables().cables
        self.assertEqual(
            (cable.part, cable.head_ref, cable.main_ref, cable.head_no_connect),
            ("test-cable", "J1", "J2", ("D1",)),
        )

    def test_head_no_connect_is_optional(self):
        model = self.cables(lambda s: s.replace('head_no_connect = ["D1"]\n', ""))
        self.assertEqual(model.cables[0].head_no_connect, ())

    def test_the_harness_view_has_no_cables(self):
        self.assertEqual(self.cables(view="harness").cables, ())

    def test_a_cable_must_be_carrier_only(self):
        for build in ('build = "harness"\n', ""):
            for view in ("harness", "carrier"):
                with (
                    self.subTest(build=build, view=view),
                    self.assertRaisesRegex(HardwareError, 'build = "carrier"'),
                ):
                    self.cables(
                        lambda s, b=build: s.replace('build = "carrier"\n', b), view
                    )

    def test_an_unknown_cable_key_fails(self):
        with self.assertRaisesRegex(HardwareError, r"unknown key\(s\) \['pins'\]"):
            self.cables(lambda s: s + 'pins = ["D0"]\n')

    def test_a_cable_needs_a_registry_part(self):
        with self.assertRaisesRegex(HardwareError, "missing or non-string 'part'"):
            self.cables(lambda s: s.replace('part = "test-cable"\n', ""))

    def test_cable_refs_are_reference_designators(self):
        with self.assertRaisesRegex(HardwareError, "head_ref 'J-1' does not match"):
            self.cables(lambda s: s.replace('head_ref = "J1"', 'head_ref = "J-1"'))

    def test_the_two_ends_need_two_refs(self):
        with self.assertRaisesRegex(
            HardwareError, "head_ref and main_ref are both 'J1'"
        ):
            self.cables(lambda s: s.replace('main_ref = "J2"', 'main_ref = "J1"'))

    def test_a_cable_ref_cannot_be_a_parts_ref(self):
        def edit(s: str) -> str:
            return s.replace('kind = "led"', 'kind = "led"\nref = "J2"')

        with self.assertRaisesRegex(
            HardwareError, r"ref 'J2' is used by \[parts.led\] and \[\[cables\]\] #1"
        ):
            self.cables(edit)

    def test_one_cable_per_carrier(self):
        second = CABLE.replace('"J1"', '"J3"').replace('"J2"', '"J4"')
        with self.assertRaisesRegex(HardwareError, r"#2: one \[\[cables\]\] entry"):
            self.cables(lambda s: s + second)

    def test_head_no_connect_must_be_pads_on_the_board(self):
        with self.assertRaisesRegex(HardwareError, "board.md has no pad named 'D9'"):
            self.cables(lambda s: s.replace('["D1"]', '["D9"]'))

    def test_head_no_connect_lists_a_pad_once(self):
        with self.assertRaisesRegex(
            HardwareError, r"head_no_connect lists \['D1'\] twice"
        ):
            self.cables(lambda s: s.replace('["D1"]', '["D1", "D1"]'))

    def test_a_head_no_connect_pad_on_a_net_fails(self):
        # D0 is LED_PIN's pad: leaving it unconnected on the head cuts that net.
        with self.assertRaisesRegex(HardwareError, "mcu.D0.*role LED_PIN"):
            self.cables(lambda s: s.replace('["D1"]', '["D0"]'))

    def test_a_head_no_connect_pad_on_a_rail_fails(self):
        with self.assertRaisesRegex(HardwareError, "mcu.5V.*rail 5V"):
            self.cables(lambda s: s.replace('["D1"]', '["5V"]') + POWER)


class RobocarUnifiedJoinTest(unittest.TestCase):
    """The committed sidecar resolves against the committed header and board."""

    @classmethod
    def setUpClass(cls):
        cls.model = join(UNIFIED)

    def test_dropping_an_undrawn_entry_from_the_committed_sidecar_fails(self):
        # The completeness gate (#460) holds on the real project, not only on
        # the synthetic one: UART0_TX_PIN is excused only by this entry.
        text = (UNIFIED / "hardware.toml").read_text(encoding="utf-8")
        entry = re.search(
            r'\[\[undrawn\]\]\nrole = "UART0_TX_PIN"\nwhy = "[^"\n]*"\n\n?', text
        )
        self.assertIsNotNone(entry)
        with tempfile.TemporaryDirectory() as tmp:
            proj = Path(tmp) / "unified"
            shutil.copytree(UNIFIED / "main", proj / "main")
            (proj / "hardware.toml").write_text(
                text.replace(entry.group(0), ""), encoding="utf-8"
            )
            with self.assertRaisesRegex(HardwareError, "UART0_TX_PIN"):
                join(proj, repo_root=REPO_ROOT)

    def test_moving_the_piezo_onto_the_mux_reset_fails(self):
        # #722's repro: PIEZO_PIN GPIO_NUM_2 -> GPIO_NUM_3, which is now
        # TCA9548A_RST_PIN's pin (#782; it was the ultrasonic trigger's). Every
        # generator used to accept it; the router failed later.
        header = (UNIFIED / "main/pin_config.h").read_text(encoding="utf-8")
        moved = header.replace(
            "#define PIEZO_PIN GPIO_NUM_2", "#define PIEZO_PIN GPIO_NUM_3"
        )
        self.assertNotEqual(moved, header)
        with tempfile.TemporaryDirectory() as tmp:
            proj = Path(tmp) / "unified"
            shutil.copytree(UNIFIED / "main", proj / "main")
            shutil.copy(UNIFIED / "hardware.toml", proj / "hardware.toml")
            (proj / "main/pin_config.h").write_text(moved, encoding="utf-8")
            with self.assertRaises(HardwareError) as ctx:
                join(proj, repo_root=REPO_ROOT)
        message = str(ctx.exception)
        self.assertIn("PIEZO_PIN", message)
        self.assertIn("TCA9548A_RST_PIN", message)
        self.assertIn("GPIO3", message)

    def test_i2s_lands_on_the_amp_via_d8_d9_d10(self):
        nets = {n.role: (n.part, n.pin) for n in self.model.nets}
        self.assertEqual(nets["I2S_BCLK_PIN"], ("amp", "BCLK"))
        self.assertEqual(self.model.pin_for("I2S_BCLK_PIN").name, "D8")
        self.assertEqual(self.model.pin_for("I2S_LRCLK_PIN").name, "D9")
        self.assertEqual(self.model.pin_for("I2S_DIN_PIN").name, "D10")

    def test_the_microphone_is_excused_and_on_no_header_pad(self):
        undrawn = {u.role for u in self.model.undrawn}
        mic = {"MIC_PDM_CLK_PIN", "MIC_PDM_DATA_PIN"}
        self.assertLessEqual(mic, undrawn)
        for role in mic:
            self.assertIsNone(self.model.pin_for(role), role)

    def test_the_uart0_pads_are_excused_and_sit_on_d6_d7(self):
        # Promoted from a doc comment to real macros (#461) so the build guide
        # and WIRING.md stop hand-typing GPIO43/44.
        undrawn = {u.role for u in self.model.undrawn}
        self.assertLessEqual({"UART0_TX_PIN", "UART0_RX_PIN"}, undrawn)
        self.assertEqual(self.model.pin_for("UART0_TX_PIN").name, "D6")
        self.assertEqual(self.model.pin_for("UART0_RX_PIN").name, "D7")

    def rail_of(self) -> dict[tuple[str, str], str]:
        return {(e.part, e.pin): r.name for r in self.model.rails for e in r.loads}

    def test_logic_supplies_are_on_3v3_and_only_vm_and_v_plus_on_5v(self):
        # WIRING.md § Logic rails are 3.3 V: both parts set their input
        # threshold from their own VCC, so a 5 V VCC strands a 3.3 V GPIO.
        rail = self.rail_of()
        self.assertEqual(rail["motor_driver", "VCC"], "3V3")
        self.assertEqual(rail["pwm", "VCC"], "3V3")
        self.assertEqual(rail["motor_driver", "VM"], "5V")
        self.assertEqual(rail["pwm", "V+"], "5V")

    def test_the_amp_and_the_mcu_are_fed_from_the_regulator(self):
        (five,) = [r for r in self.model.rails if r.name == "5V"]
        self.assertEqual((five.source.part, five.source.pin), ("buck", "OUT+"))
        rail = self.rail_of()
        self.assertEqual(rail["amp", "Vin"], "5V")
        self.assertEqual(rail["mcu", "5V"], "5V")

    def test_the_3v3_rail_comes_from_the_xiaos_3v3_pad(self):
        (three,) = [r for r in self.model.rails if r.name == "3V3"]
        self.assertEqual((three.source.part, three.source.pin), ("mcu", "3V3"))

    def test_every_channel_the_firmware_drives_is_on_a_channel_net(self):
        # A channel added to pin_config.h without a [[channel_nets]] entry
        # would draw as a bare pad number; an alias of a wired channel
        # (MOTOR_FIRST_CHANNEL) needs no net of its own.
        wired = {n.channel for n in self.model.channel_nets if n.source == "pwm"}
        self.assertEqual(wired, set(self.model.channels.values()))
        # By role too: a new macro on an already-wired channel (a second role
        # on channel 6 beside SERVO_PAN) passes the number check above.
        unwired = set(self.model.channels) - {n.role for n in self.model.channel_nets}
        self.assertEqual(unwired, {"MOTOR_FIRST_CHANNEL"})

    def test_the_motor_driver_mapping_agrees_with_pin_config_comments(self):
        # pin_config.h states the PCA9685 -> TB6612FNG mapping in comments
        # (`// -> PWMA`); hardware.toml states it as nets. Pin the two together.
        text = (UNIFIED / "main/pin_config.h").read_text(encoding="utf-8")
        claimed = dict(re.findall(r"#define (\w+_CHANNEL) \d+\s*// -> (\w+)", text))
        joined = {
            n.role: n.pin for n in self.model.channel_nets if n.part == "motor_driver"
        }
        self.assertEqual(len(claimed), 6)
        self.assertEqual(claimed, joined)

    def test_every_mux_channel_the_firmware_selects_is_on_a_bus_net(self):
        # pin_config.h: the TCA9548A's five channels, each to the part the
        # firmware selects it for (#768, #782): ten wires in all. The two
        # rangefinders share an address, so each has a channel to itself.
        self.assertEqual(
            {b.role: (b.source, b.channel, b.part) for b in self.model.bus_nets},
            {
                "I2C_BUS_CHANNEL_PCA9685": ("mux", 0, "pwm"),
                "I2C_BUS_CHANNEL_OLED": ("mux", 1, "oled"),
                "I2C_BUS_CHANNEL_MCP23017": ("mux", 2, "expander"),
                "I2C_BUS_CHANNEL_RANGER_FRONT": ("mux", 3, "ranger_front"),
                "I2C_BUS_CHANNEL_RANGER_REAR": ("mux", 4, "ranger_rear"),
            },
        )
        roles = {b.role for b in self.model.bus_nets}
        self.assertEqual(set(self.model.bus_channels), roles)
        self.assertEqual(len([w for b in self.model.bus_nets for w in b.wires]), 10)

    def test_each_motor_takes_its_own_driver_channel(self):
        # sparkfun-tb6612fng.md: channel A (A01/A02) is the right motor, B the
        # left. Without load_pins both motors would share all four nets (#769).
        (motors,) = [o for o in self.model.outputs if o.part == "motor_driver"]
        self.assertEqual(
            motors.wires,
            (
                ("A01", "motor_right", "1"),
                ("A02", "motor_right", "2"),
                ("B01", "motor_left", "1"),
                ("B02", "motor_left", "2"),
            ),
        )

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

    def test_balancebot_convention_emits_its_own_extract_list(self):
        # A stand-in project: balancebot's real header, every role undrawn, so
        # the test does not depend on a hardware.toml in the project itself.
        with tempfile.TemporaryDirectory() as tmp:
            proj = Path(tmp) / "bb"
            (proj / "src").mkdir(parents=True)
            shutil.copy(BALANCEBOT / "src/pin_config.h", proj / "src/pin_config.h")
            roles = roles_from_defines(
                parse_defines([proj / "src/pin_config.h"]), "balancebot"
            )
            sidecar = textwrap.dedent("""\
                [source]
                convention = "balancebot"
                header = "src/pin_config.h"
                board = "docs/reference/boards/xiao-rp2350.md"
                """) + "".join(
                f'\n[[undrawn]]\nrole = "{r}"\nwhy = "test"\n' for r in sorted(roles)
            )
            (proj / "hardware.toml").write_text(sidecar)
            text = self.run_generator(REPO_ROOT, str(proj)).decode()
        self.assertIn("#let PIN_IMU_SDA = 6\n", text)
        self.assertIn("#let PIN_MOTOR_NENABLE = 1\n", text)
        self.assertIn("#let PIN_BATT_ADC_ENABLE = 19\n", text)
        self.assertNotIn("I2C_SDA_PIN", text)
        # Not mounted in the root justfile, so it names the script itself.
        self.assertIn(
            "regenerate with `python3 tools/typst/generate-pin-defs.py ", text
        )

    def test_the_regenerate_line_names_the_projects_just_module(self):
        spec = importlib.util.spec_from_file_location("gen_pin_defs", GENERATOR)
        gen = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(gen)
        self.assertEqual(
            gen.regenerate_command(BALANCEBOT), "just balancebot::gen-pin-defs"
        )
        self.assertEqual(
            gen.regenerate_command(UNIFIED), "just robocar-unified::gen-pin-defs"
        )


if __name__ == "__main__":
    unittest.main()
