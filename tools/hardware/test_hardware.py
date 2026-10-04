"""Tests for tools/hardware — the board × header × parts join (ADR-021, #459).

Stdlib `unittest` only, so the suite runs wherever the generator does (CI's
system python3, the justfile, the pre-commit hook) with nothing to install:

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import os
import re
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

    def test_a_part_board_is_kept_and_defaults_to_none(self):
        self.assertEqual(self.join(self.make()).parts["led"].board, "")
        sidecar = SIDECAR.replace('kind = "led"', 'kind = "led"\nboard = "board.md"')
        self.assertEqual(
            self.join(self.make(sidecar=sidecar)).parts["led"].board, "board.md"
        )

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
