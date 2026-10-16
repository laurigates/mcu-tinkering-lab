"""Tests for the carrier netlist built from the hardware join (ADR-027).

The netlist built from the frozen balancebot inputs (`kicad.testfixture`) must
equal `testdata/balancebot-carrier.net.json`, which is `parity.normalise` of
KiCad's own netlist export of the #749 schematic
(`testdata/balancebot-carrier.xml`). The live project is not read here: its
drift is `kicad.schematic --check`'s and the check stamp's job. Mutations run
on a temp copy of the frozen inputs.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import json
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from kicad import netlist, testfixture
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

ROOT, PROJECT = testfixture.shared()
TESTDATA = Path(__file__).resolve().parent / "testdata"
FIXTURE = TESTDATA / "balancebot-carrier.net.json"
EXPORT = TESTDATA / "balancebot-carrier.xml"


def fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


class BalancebotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT, ROOT)

    def test_equals_the_749_export(self):
        self.assertEqual(netlist.as_dict(self.nl), fixture())

    def test_counts(self):
        self.assertEqual(
            (len(self.nl.comps), len(self.nl.nets), len(self.nl.no_connect)),
            (19, 22, 6),
        )

    def test_ordering_is_kicads(self):
        refs = [c.ref for c in self.nl.comps]
        self.assertEqual(refs, sorted(refs, key=netlist.ref_key))
        self.assertEqual(list(self.nl.nets), sorted(self.nl.nets))
        # Pins sort as plain strings, as KiCad's export does: 1, 12, 7.
        self.assertEqual(
            [p for r, p in self.nl.nets["GND"] if r == "A1"], ["1", "12", "7"]
        )
        root = ET.parse(EXPORT).getroot()
        self.assertEqual(refs, [c.get("ref") for c in root.iter("comp")])
        self.assertEqual(
            list(self.nl.nets) + list(netlist.unconnected(self.nl)),
            [n.get("name") for n in root.find("nets")],
        )

    def test_component_fields(self):
        comps = {c.ref: c for c in self.nl.comps}
        self.assertEqual(comps["U1"].value, "XIAO RP2350")
        self.assertEqual(comps["U1"].part_key, "mcu")
        self.assertEqual(comps["R1"].fields, (("LCSC", "C17414"),))
        self.assertEqual((comps["D1"].value, comps["D1"].lcsc), ("B5819W", "C8598"))
        self.assertEqual(comps["J3"].value, "GY-521 MPU6050")  # the part's name
        self.assertEqual((comps["H1"].in_bom, comps["H1"].on_board), (False, True))
        self.assertEqual(comps["A1"].registry, "pololu-drv8825")

    def test_cli(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(netlist.main([str(PROJECT)], ROOT), 0)
        self.assertIn("19 components, 22 nets, 6 no-connects", out.getvalue())
        self.assertIn("unconnected-(A1-~{FLT}-Pad2): A1.2", out.getvalue())


class NamingTests(unittest.TestCase):
    def test_nc_net_names_match_kicads(self):
        root = ET.parse(EXPORT).getroot()
        kicad = sorted(
            n.get("name")
            for n in root.find("nets")
            if n.get("name").startswith("unconnected-")
        )
        nl = netlist.build(PROJECT, ROOT)
        self.assertEqual(sorted(netlist.unconnected(nl)), kicad)
        self.assertEqual(
            netlist.nc_net_name("A1", "~{FLT}", "2"), "unconnected-(A1-~{FLT}-Pad2)"
        )
        self.assertEqual(netlist.nc_net_name("C1", "", "1"), "unconnected-(C1-Pad1)")

    def test_ref_sort(self):
        self.assertLess(netlist.ref_key("R2"), netlist.ref_key("R10"))
        self.assertEqual(netlist.ref_key("SW1"), ("SW", 1))
        self.assertEqual(netlist.ref_key("#FLG01"), ("#FLG", 1))
        self.assertEqual(
            sorted(["R10", "SW1", "R2", "C1"], key=netlist.ref_key),
            ["C1", "R2", "R10", "SW1"],
        )
        with self.assertRaisesRegex(KicadError, "not a reference designator"):
            netlist.ref_key("U1A")


class MutationTests(unittest.TestCase):
    """A temp copy of the frozen inputs; each edit must change or fail the build."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.project = testfixture.materialise(self.root)
        self.toml = self.project / "hardware.toml"
        self.header = self.project / "src/pin_config.h"

    def build(self) -> netlist.Netlist:
        return netlist.build(self.project, self.root)

    def edit(self, path: Path, old: str, new: str) -> None:
        text = path.read_text(encoding="utf-8")
        self.assertEqual(text.count(old), 1, old)
        path.write_text(text.replace(old, new), encoding="utf-8")

    def fails(self, msg: str) -> None:
        with self.assertRaisesRegex(KicadError, msg):
            self.build()

    def test_unedited_copy_equals_the_fixture(self):
        self.assertEqual(netlist.as_dict(self.build()), fixture())

    def test_imu_ad0_off_gnd(self):
        self.edit(self.toml, '    "imu.AD0",\n', "")
        self.fails(r"J3\.7 \(AD0\) unconnected, not declared")

    def test_dropped_net_name_renames_the_net(self):
        self.edit(self.toml, 'PIN_STEP_LEFT = "STEP_L"\n', "")
        nets = self.build().nets
        self.assertIn("PIN_STEP_LEFT", nets)
        self.assertNotIn("STEP_L", nets)
        self.assertNotEqual(netlist.as_dict(self.build()), fixture())

    def test_same_pin_on_two_nets(self):
        # Two roles onto one pin is now the join's error (#722), so this
        # reaches the netlist's own check through what the join lets past:
        # one load driven by two drivers' outputs.
        self.edit(
            self.toml,
            'to = ["stepper_right"]',
            'to = ["stepper_right", "stepper_left"]',
        )
        self.fails(r"J4\.1 is on net 'MOTL_A1' and net 'MOTR_A1'")

    def test_two_roles_on_one_pin_fail_in_the_join(self):
        self.edit(
            self.toml,
            '[[nets]]\nrole = "PIN_IMU_INT"\n',
            '[[nets]]\nrole = "PIN_IMU_INT"\nto = "imu.SDA"\n\n[[nets]]\nrole = "PIN_IMU_INT"\n',
        )
        self.fails(r"imu\.SDA is driven by both PIN_IMU_SDA and PIN_IMU_INT")

    def test_part_without_ref(self):
        self.edit(self.toml, 'part = "gy-521"\nref = "J3"\n', 'part = "gy-521"\n')
        self.fails(r"\[parts\.imu\]: the carrier needs `part` and `ref`")

    def test_mcu_without_ref(self):
        self.edit(self.toml, 'ref = "U1"\n', "")
        self.fails(r"\[source\]: the carrier needs `part` and `ref` for the MCU")

    def test_mcu_role_off_the_header(self):
        self.edit(self.header, "#define PIN_IMU_INT 5 ", "#define PIN_IMU_INT 21 ")
        self.fails(r"role PIN_IMU_INT \(GPIO21\) is not on a header pad")

    def test_moved_pin_moves_the_net(self):
        self.edit(self.header, "#define PIN_DIR_LEFT 4 ", "#define PIN_DIR_LEFT 27 ")
        nl = self.build()
        self.assertEqual(nl.nets["DIR_L"], (("A1", "16"), ("U1", "2")))
        self.assertIn(("U1", "10"), nl.no_connect)

    def test_mcu_gnd_off_its_rail(self):
        self.edit(self.toml, '    "mcu.GND",\n', "")
        self.fails(r"U1\.13 \(GND\) is a power pad and is on no rail")

    def test_mcu_5v_off_its_rail(self):
        self.edit(
            self.toml,
            '[[rails]]\nname = "XIAO_5V"\nbuild = "carrier"\nfrom = "buck_diode.K"\nto = ["mcu.5V"]\n',
            "",
        )
        self.fails(r"U1\.14 \(5V\) is a power pad and is on no rail")

    def test_unknown_pin_name(self):
        self.edit(self.toml, 'to = "imu.SDA"', 'to = "imu.SDAX"')
        self.fails(r"imu\.SDAX: gy-521\.toml has no pin 'SDAX'")

    def test_unknown_registry_entry(self):
        self.edit(self.toml, 'part = "gy-521"', 'part = "gy-522"')
        self.fails("no parts-registry entry 'gy-522'")

    def test_output_without_pins(self):
        self.edit(
            self.toml, 'pins = ["A1", "A2", "B1", "B2"]\nnet = "MOTL_{pin}"\n', ""
        )
        self.fails(r"\[\[outputs\]\] from 'drv_left' has no `pins`")

    def test_registry_value_and_lcsc_are_defaults(self):
        self.edit(self.toml, 'value = "10k"\nlcsc = "C17414"\n', "")
        r1 = {c.ref: c for c in self.build().comps}["R1"]
        self.assertEqual((r1.value, r1.fields), ("10 kΩ resistor", ()))

    def test_join_errors_become_kicad_errors(self):
        self.edit(self.toml, 'role = "PIN_IMU_SDA"', 'role = "PIN_IMU_SDAX"')
        self.fails("PIN_IMU_SDAX")


# --- Mux channel buses (#768) ---------------------------------------------------
# A stand-in project on the frozen tree's XIAO and registry: robocar-unified's
# own I2C and TCA9548A channel macros, a mux on a 1x8 socket and three 2-pin
# loads. A stand-in rather than the real project, so a change to its
# hardware.toml cannot move these cases.

UNIFIED_HEADER = REPO_ROOT / "packages/robocar/unified/main/pin_config.h"

MUX_PARTS = {
    "test-mux": """\
[part]
name = "Test I2C mux"
source = "test stand-in (#768)"
symbol = "Connector_Generic:Conn_01x08"
footprint = "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical"

[pins]
SDA = 1
SCL = 2
SD0 = 3
SC0 = 4
SD1 = 5
SC1 = 6
SD2 = 7
SC2 = 8
""",
    "test-i2c-load": """\
[part]
name = "Test I2C load"
source = "test stand-in (#768)"
symbol = "Connector_Generic:Conn_01x02"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"

[pins]
SDA = 1
SCL = 2
""",
    "test-power": """\
[part]
name = "Test power lead"
source = "test stand-in (#768)"
symbol = "Connector_Generic:Conn_01x04"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"
no_connect = ["NC"]

[pins]
"5V" = 1
GND = 2
"3V3" = 3
NC = 4
""",
}

MUX_SIDECAR = """\
[source]
convention = "robocar"
header = "main/pin_config.h"
board = "docs/reference/boards/xiao-rp2350.md"
part = "seeed-xiao-rp2350-socket"
ref = "U1"

[parts.mux]
name = "TCA9548A"
kind = "i2c-mux"
part = "test-mux"
ref = "J1"

[parts.pwm]
name = "PCA9685"
kind = "pwm-driver"
part = "test-i2c-load"
ref = "J2"

[parts.oled]
name = "SSD1306"
kind = "display"
part = "test-i2c-load"
ref = "J3"

[parts.expander]
name = "MCP23017"
kind = "gpio-expander"
part = "test-i2c-load"
ref = "J4"

[parts.power]
name = "Power lead"
kind = "connector"
part = "test-power"
ref = "J5"

[[nets]]
role = "I2C_SDA_PIN"
to = "mux.SDA"

[[nets]]
role = "I2C_SCL_PIN"
to = "mux.SCL"

[[bus_nets]]
role = "I2C_BUS_CHANNEL_PCA9685"
from = "mux"
to = "pwm"

[[bus_nets]]
role = "I2C_BUS_CHANNEL_OLED"
from = "mux"
to = "oled"

[[bus_nets]]
role = "I2C_BUS_CHANNEL_MCP23017"
from = "mux"
to = "expander"

[[rails]]
name = "5V"
from = "mcu.5V"
to = ["power.5V"]

[[rails]]
name = "GND"
from = "mcu.GND"
to = ["power.GND"]

[[rails]]
name = "3V3"
from = "mcu.3V3"
to = ["power.3V3"]
"""


class BusNetTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        testfixture.materialise(self.root)
        for slug, text in MUX_PARTS.items():
            (self.root / f"docs/reference/parts/{slug}.toml").write_text(
                text, encoding="utf-8"
            )
        self.project = self.root / "packages/test/muxed"
        (self.project / "main").mkdir(parents=True)
        # The real header's I2C and mux-channel macros, extracted, not retyped.
        wanted = re.compile(r"#define (I2C_SDA_PIN|I2C_SCL_PIN|I2C_BUS_CHANNEL_\w+) ")
        text = UNIFIED_HEADER.read_text(encoding="utf-8")
        lines = [line for line in text.splitlines() if wanted.match(line)]
        # SDA, SCL and five mux channels (#782 added the two rangefinders').
        # MUX_SIDECAR wires three of them; a bus channel role need not be wired.
        self.assertEqual(len(lines), 7, lines)
        self.header = self.project / "main/pin_config.h"
        self.header.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.toml = self.project / "hardware.toml"
        self.toml.write_text(MUX_SIDECAR, encoding="utf-8")

    def build(self) -> netlist.Netlist:
        return netlist.build(self.project, self.root)

    def buses(self, nl: netlist.Netlist) -> dict[str, tuple[tuple[str, str], ...]]:
        return {k: v for k, v in nl.nets.items() if "BUS_CHANNEL" in k}

    def test_each_mux_channel_is_an_sda_and_an_scl_net(self):
        self.assertEqual(
            self.buses(self.build()),
            {
                "I2C_BUS_CHANNEL_MCP23017_SCL": (("J1", "8"), ("J4", "2")),
                "I2C_BUS_CHANNEL_MCP23017_SDA": (("J1", "7"), ("J4", "1")),
                "I2C_BUS_CHANNEL_OLED_SCL": (("J1", "6"), ("J3", "2")),
                "I2C_BUS_CHANNEL_OLED_SDA": (("J1", "5"), ("J3", "1")),
                "I2C_BUS_CHANNEL_PCA9685_SCL": (("J1", "4"), ("J2", "2")),
                "I2C_BUS_CHANNEL_PCA9685_SDA": (("J1", "3"), ("J2", "1")),
            },
        )

    def test_the_upstream_bus_stays_an_mcu_net(self):
        nl = self.build()
        self.assertIn(("J1", "1"), nl.nets["I2C_SDA_PIN"])
        self.assertIn(("J1", "2"), nl.nets["I2C_SCL_PIN"])

    def test_a_renumbered_channel_moves_its_nets(self):
        # pin_config.h swaps the OLED and the expander between channels 1 and 2.
        swap = {"OLED": "2", "MCP23017": "1"}
        text = re.sub(
            r"(I2C_BUS_CHANNEL_(OLED|MCP23017)) \d",
            lambda m: f"{m.group(1)} {swap[m.group(2)]}",
            self.header.read_text(encoding="utf-8"),
        )
        self.header.write_text(text, encoding="utf-8")
        nets = self.buses(self.build())
        self.assertEqual(nets["I2C_BUS_CHANNEL_OLED_SDA"], (("J1", "7"), ("J3", "1")))
        self.assertEqual(
            nets["I2C_BUS_CHANNEL_MCP23017_SCL"], (("J1", "6"), ("J4", "2"))
        )

    def test_net_names_rename_both_lines(self):
        with self.toml.open("a", encoding="utf-8") as f:
            f.write('\n[net_names]\nI2C_BUS_CHANNEL_OLED = "OLED"\n')
        nl = self.build()
        self.assertNotIn("I2C_BUS_CHANNEL_OLED_SDA", nl.nets)
        self.assertEqual(nl.nets["OLED_SDA"], (("J1", "5"), ("J3", "1")))
        self.assertEqual(nl.nets["OLED_SCL"], (("J1", "6"), ("J3", "2")))

    def test_a_one_line_load_joins_the_net_named_for_its_line(self):
        # A pull-up per line (#750): the resistor's pin B joins the channel's
        # <name>_SDA / _SCL, named by the line, not by the resistor's pin.
        parts = "".join(
            f'\n[parts.pull_{x}]\nname = "10k"\nkind = "resistor"\n'
            f'part = "r0805"\nref = "R{n}"\nvalue = "10k"\n'
            for n, x in ((1, "sda"), (2, "scl"))
        )
        buses = "".join(
            f'\n[[bus_nets]]\nrole = "I2C_BUS_CHANNEL_OLED"\nfrom = "mux"\n'
            f'to = "pull_{x.lower()}.B"\nline = "{x}"\n'
            for x in ("SDA", "SCL")
        )
        rail = '\n[[rails]]\nname = "3V3"\nto = ["pull_sda.A", "pull_scl.A"]\n'
        with self.toml.open("a", encoding="utf-8") as f:
            f.write(parts + buses + rail)
        nets = self.buses(self.build())
        self.assertEqual(
            nets["I2C_BUS_CHANNEL_OLED_SDA"], (("J1", "5"), ("J3", "1"), ("R1", "2"))
        )
        self.assertEqual(
            nets["I2C_BUS_CHANNEL_OLED_SCL"], (("J1", "6"), ("J3", "2"), ("R2", "2"))
        )
        self.assertNotIn("I2C_BUS_CHANNEL_OLED_B", nets)

    def test_a_dropped_bus_net_leaves_its_pins_unconnected(self):
        text = self.toml.read_text(encoding="utf-8")
        entry = (
            '[[bus_nets]]\nrole = "I2C_BUS_CHANNEL_OLED"\nfrom = "mux"\nto = "oled"\n'
        )
        self.assertEqual(text.count(entry), 1)
        self.toml.write_text(text.replace(entry, ""), encoding="utf-8")
        with self.assertRaisesRegex(KicadError, r"J1\.5 \(SD1\) unconnected"):
            self.build()

    def test_a_bus_pin_missing_from_the_registry_fails(self):
        mux = self.root / "docs/reference/parts/test-mux.toml"
        mux.write_text(
            mux.read_text(encoding="utf-8").replace("SC2 = 8", "XX = 8"),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(KicadError, r"mux\.SC2: test-mux\.toml has no pin"):
            self.build()


# --- One driver, two loads on separate pins (#769) -------------------------------
# robocar-unified's TB6612FNG drives the right motor from A01/A02 and the left
# from B01/B02 (sparkfun-tb6612fng.md), and both motors use one 2-pin connector
# entry. A stand-in project on the frozen tree's XIAO and registry, so a change
# to the real project's hardware.toml cannot move these cases.

LOAD_PIN_PARTS = {
    "test-motor-driver": """\
[part]
name = "Test motor driver outputs"
source = "test stand-in (#769)"
symbol = "Connector_Generic:Conn_01x04"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"

[pins]
A01 = 1
A02 = 2
B01 = 3
B02 = 4
""",
    "test-motor": """\
[part]
name = "Test DC motor lead"
source = "test stand-in (#769)"
symbol = "Connector_Generic:Conn_01x02"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"

[pins]
"1" = 1
"2" = 2
""",
    "test-power": """\
[part]
name = "Test power lead"
source = "test stand-in (#769)"
symbol = "Connector_Generic:Conn_01x04"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"
no_connect = ["NC"]

[pins]
"5V" = 1
GND = 2
"3V3" = 3
NC = 4
""",
}

LOAD_PIN_SIDECAR = """\
[source]
convention = "robocar"
header = "main/pin_config.h"
board = "docs/reference/boards/xiao-rp2350.md"
part = "seeed-xiao-rp2350-socket"
ref = "U1"

[parts.motor_driver]
name = "TB6612FNG"
kind = "motor-driver"
part = "test-motor-driver"
ref = "J1"

[parts.motor_left]
name = "Left motor"
kind = "dc-motor"
part = "test-motor"
ref = "J2"

[parts.motor_right]
name = "Right motor"
kind = "dc-motor"
part = "test-motor"
ref = "J3"

[parts.power]
name = "Power lead"
kind = "connector"
part = "test-power"
ref = "J4"

[[rails]]
name = "5V"
from = "mcu.5V"
to = ["power.5V"]

[[rails]]
name = "GND"
from = "mcu.GND"
to = ["power.GND"]

[[rails]]
name = "3V3"
from = "mcu.3V3"
to = ["power.3V3"]

[[outputs]]
from = "motor_driver"
to = ["motor_left", "motor_right"]
pins = ["A01", "A02", "B01", "B02"]
net = "MOT_{pin}"
load_pins.motor_right = { A01 = "1", A02 = "2" }
load_pins.motor_left = { B01 = "1", B02 = "2" }
"""


class LoadPinTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        testfixture.materialise(self.root)
        for slug, text in LOAD_PIN_PARTS.items():
            (self.root / f"docs/reference/parts/{slug}.toml").write_text(
                text, encoding="utf-8"
            )
        self.project = self.root / "packages/test/motors"
        (self.project / "main").mkdir(parents=True)
        (self.project / "main/pin_config.h").write_text(
            "// no pin roles\n", encoding="utf-8"
        )
        self.toml = self.project / "hardware.toml"
        self.toml.write_text(LOAD_PIN_SIDECAR, encoding="utf-8")

    def build(self) -> netlist.Netlist:
        return netlist.build(self.project, self.root)

    def test_each_motor_is_on_its_own_pair_of_nets(self):
        nets = {k: v for k, v in self.build().nets.items() if k.startswith("MOT_")}
        self.assertEqual(
            nets,
            {
                "MOT_A01": (("J1", "1"), ("J3", "1")),
                "MOT_A02": (("J1", "2"), ("J3", "2")),
                "MOT_B01": (("J1", "3"), ("J2", "1")),
                "MOT_B02": (("J1", "4"), ("J2", "2")),
            },
        )

    def test_without_load_pins_a_load_needs_the_drivers_pin_names(self):
        # Pin p to pin p of every load: the motor connector has no pin A01.
        text = self.toml.read_text(encoding="utf-8")
        mapping = (
            'load_pins.motor_right = { A01 = "1", A02 = "2" }\n'
            'load_pins.motor_left = { B01 = "1", B02 = "2" }\n'
        )
        self.assertEqual(text.count(mapping), 1)
        self.toml.write_text(text.replace(mapping, ""), encoding="utf-8")
        with self.assertRaisesRegex(
            KicadError, r"motor_left\.A01: test-motor\.toml has no pin 'A01'"
        ):
            self.build()

    def test_a_load_pin_missing_from_the_registry_fails(self):
        text = self.toml.read_text(encoding="utf-8")
        self.toml.write_text(
            text.replace('{ B01 = "1", B02 = "2" }', '{ B01 = "1", B02 = "3" }'),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(
            KicadError, r"motor_left\.3: test-motor\.toml has no pin '3'"
        ):
            self.build()


# --- A cable between the head board and the main board (#750) --------------------
# robocar-unified's XIAO ESP32-S3 rides on the pan-tilt head and reaches the main
# board through a ribbon cable. A stand-in project: the real S3 socket entry and
# board page, the real header's I2C and UART0 macros, and an 8-way cable whose
# [pins] are keyed by the XIAO's pad names.

S3_SOCKET = "seeed-xiao-esp32s3-socket"
S3_BOARD = "docs/reference/boards/xiao-esp32s3.md"

CABLE_PARTS = {
    "test-cable": """\
[part]
name = "Test head cable"
source = "test stand-in (#750)"
symbol = "Connector_Generic:Conn_01x08"
footprint = "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical"

[pins]
D4 = 1
GND = [2, 6, 7]
D5 = 3
"5V" = [4, 5, 8]
""",
    "test-i2c-load": MUX_PARTS["test-i2c-load"],
    "test-lead": """\
[part]
name = "Test power lead"
source = "test stand-in (#750)"
symbol = "Connector_Generic:Conn_01x02"
footprint = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"

[pins]
"5V" = 1
GND = 2
""",
}

CABLE_SIDECAR = f"""\
[source]
convention = "robocar"
header = "main/pin_config.h"
board = "{S3_BOARD}"
part = "{S3_SOCKET}"
ref = "U1"

[parts.sensor]
name = "I2C sensor"
kind = "sensor"
part = "test-i2c-load"
ref = "J3"

[parts.power]
name = "Power lead"
kind = "connector"
part = "test-lead"
ref = "J4"

[[nets]]
role = "I2C_SDA_PIN"
to = "sensor.SDA"

[[nets]]
role = "I2C_SCL_PIN"
to = "sensor.SCL"

[[undrawn]]
role = "UART0_TX_PIN"
why = "stays on the head"

[[undrawn]]
role = "UART0_RX_PIN"
why = "stays on the head"

[[rails]]
name = "5V"
from = "power.5V"
to = ["mcu.5V"]

[[rails]]
name = "GND"
from = "power.GND"
to = ["mcu.GND"]

[[cables]]
part = "test-cable"
head_ref = "J1"
main_ref = "J2"
head_no_connect = ["3V3", "D6", "D7"]
build = "carrier"
"""


class CableTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        testfixture.materialise(self.root)
        for rel in (S3_BOARD, f"docs/reference/parts/{S3_SOCKET}.toml"):
            (self.root / rel).write_bytes((REPO_ROOT / rel).read_bytes())
        for slug, text in CABLE_PARTS.items():
            self.registry(slug).write_text(text, encoding="utf-8")
        self.project = self.root / "packages/test/headed"
        (self.project / "main").mkdir(parents=True)
        # The real header's I2C and UART0 macros, extracted, not retyped.
        wanted = re.compile(
            r"#define (I2C_SDA_PIN|I2C_SCL_PIN|UART0_TX_PIN|UART0_RX_PIN) "
        )
        text = UNIFIED_HEADER.read_text(encoding="utf-8")
        lines = [line for line in text.splitlines() if wanted.match(line)]
        self.assertEqual(len(lines), 4, lines)
        (self.project / "main/pin_config.h").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        self.toml = self.project / "hardware.toml"
        self.toml.write_text(CABLE_SIDECAR, encoding="utf-8")

    def registry(self, slug: str) -> Path:
        return self.root / f"docs/reference/parts/{slug}.toml"

    def edit(self, path: Path, old: str, new: str) -> None:
        text = path.read_text(encoding="utf-8")
        self.assertEqual(text.count(old), 1, old)
        path.write_text(text.replace(old, new), encoding="utf-8")

    def build(self) -> netlist.Netlist:
        return netlist.build(self.project, self.root)

    def fails(self, msg: str) -> None:
        with self.assertRaisesRegex(KicadError, msg):
            self.build()

    def test_each_carried_signal_is_a_head_net_and_a_main_net(self):
        nets = self.build().nets
        # XIAO D4 is socket pin 5 and D5 pin 6; the cable carries them on pins 1, 3.
        self.assertEqual(nets["HEAD_I2C_SDA_PIN"], (("J1", "1"), ("U1", "5")))
        self.assertEqual(nets["I2C_SDA_PIN"], (("J2", "1"), ("J3", "1")))
        self.assertEqual(nets["HEAD_I2C_SCL_PIN"], (("J1", "3"), ("U1", "6")))
        self.assertEqual(nets["I2C_SCL_PIN"], (("J2", "3"), ("J3", "2")))

    def test_rails_split_at_the_cable(self):
        nets = self.build().nets
        self.assertEqual(
            nets["HEAD_5V"], (("J1", "4"), ("J1", "5"), ("J1", "8"), ("U1", "14"))
        )
        self.assertEqual(
            nets["5V"], (("J2", "4"), ("J2", "5"), ("J2", "8"), ("J4", "1"))
        )
        self.assertEqual(
            nets["HEAD_GND"], (("J1", "2"), ("J1", "6"), ("J1", "7"), ("U1", "13"))
        )
        self.assertEqual(
            nets["GND"], (("J2", "2"), ("J2", "6"), ("J2", "7"), ("J4", "2"))
        )

    def test_no_net_runs_from_the_mcu_to_the_main_board(self):
        nl = self.build()
        for name, members in nl.nets.items():
            refs = {r for r, _ in members}
            with self.subTest(net=name):
                self.assertFalse({"U1"} & refs and refs & {"J2", "J3", "J4"})

    def test_both_ends_are_one_registry_entry(self):
        comps = {c.ref: c for c in self.build().comps}
        self.assertEqual(comps["J1"].registry, "test-cable")
        self.assertEqual(comps["J2"].registry, "test-cable")
        self.assertEqual(comps["J1"].lib_id, comps["J2"].lib_id)
        self.assertEqual(comps["J1"].value, "Test head cable")

    def test_net_names_rename_both_halves(self):
        with self.toml.open("a", encoding="utf-8") as f:
            f.write('\n[net_names]\nI2C_SDA_PIN = "SDA"\n5V = "+5V"\n')
        nets = self.build().nets
        self.assertEqual(nets["HEAD_SDA"], (("J1", "1"), ("U1", "5")))
        self.assertEqual(nets["SDA"], (("J2", "1"), ("J3", "1")))
        self.assertIn("HEAD_+5V", nets)
        self.assertIn("+5V", nets)

    def test_head_no_connect_lets_a_power_pad_float(self):
        nl = self.build()
        self.assertIn(("U1", "12"), nl.no_connect)  # 3V3
        self.edit(self.toml, '["3V3", "D6", "D7"]', '["D6", "D7"]')
        self.fails(r"U1\.12 \(3V3\) is a power pad and is on no rail")

    def test_an_uncarried_role_fails(self):
        # The cable carries D4 and D5; move SCL to D8 (GPIO7), which it does not.
        self.edit(
            self.project / "main/pin_config.h",
            "#define I2C_SCL_PIN GPIO_NUM_6 ",
            "#define I2C_SCL_PIN GPIO_NUM_7 ",
        )
        self.fails(
            r"role I2C_SCL_PIN is on U1 pad D8, which cable J1-J2 \(test-cable\) "
            r"does not carry"
        )

    def test_an_uncarried_rail_pad_fails(self):
        self.edit(self.registry("test-cable"), '"5V" = [4, 5, 8]', "D8 = [4, 5, 8]")
        self.fails(r"rail 5V is on U1 pad 5V, which cable J1-J2 .* does not carry")

    def test_a_cable_pin_on_no_source_pad_fails(self):
        self.edit(self.registry("test-cable"), "D5 = 3", "SCL = 3")
        self.fails(r"test-cable\.toml \[pins\] 'SCL' is not a pad of xiao-esp32s3\.md")

    def test_a_no_connect_cable_pin_is_open_at_both_ends(self):
        # A spare conductor: named for no XIAO pad, listed under no_connect.
        cable = self.registry("test-cable")
        self.edit(cable, '"5V" = [4, 5, 8]', '"5V" = [4, 8]\nNC = 5')
        self.edit(cable, 'footprint = "', 'no_connect = ["NC"]\nfootprint = "')
        nl = self.build()
        self.assertIn(("J1", "5"), nl.no_connect)
        self.assertIn(("J2", "5"), nl.no_connect)
        self.assertNotIn(("J1", "5"), nl.nets["HEAD_5V"])

    def test_an_unlisted_spare_cable_pin_fails(self):
        self.edit(
            self.registry("test-cable"), '"5V" = [4, 5, 8]', '"5V" = [4, 8]\nNC = 5'
        )
        self.fails(r"test-cable\.toml \[pins\] 'NC' is not a pad of xiao-esp32s3\.md")

    def test_a_carried_pad_cannot_stay_unconnected_on_the_head(self):
        self.edit(self.registry("test-cable"), "D5 = 3", "D5 = 3\nD7 = 5")
        self.edit(self.registry("test-cable"), '"5V" = [4, 5, 8]', '"5V" = [4, 8]')
        self.fails(r"head_no_connect \['D7'\]: test-cable\.toml carries")

    def test_a_carried_pad_on_no_net_leaves_its_cable_pins_open(self):
        self.edit(self.registry("test-cable"), "D5 = 3", "D5 = 3\nD0 = 5")
        self.edit(self.registry("test-cable"), '"5V" = [4, 5, 8]', '"5V" = [4, 8]')
        self.fails(r"J1\.5 \(D0\) unconnected, not declared")

    def test_an_unknown_cable_entry_fails(self):
        self.edit(self.toml, 'part = "test-cable"', 'part = "test-cabel"')
        self.fails("no parts-registry entry 'test-cabel'")


# --- Board pages on carrier parts (#750 step 2) -----------------------------------
# robocar-unified's breakouts name a `board` page that documents a module's
# headers, not a GPIO pin map. Only a `pin_numbers` entry reads its page; a
# breakout's [pins] come from the registry, so its page must not be parsed.

BREAKOUT_PAGE = "docs/reference/boards/adafruit-tca9548a.md"


class BoardPageTests(unittest.TestCase):
    def setUp(self):
        BusNetTests.setUp(self)  # the same stand-in project, without its tests

    def add_board(self, part: str, page: str) -> None:
        text = self.toml.read_text(encoding="utf-8")
        old = f"[parts.{part}]\n"
        self.assertEqual(text.count(old), 1, old)
        self.toml.write_text(
            text.replace(old, f'{old}board = "{page}"\n'), encoding="utf-8"
        )

    def test_a_breakout_board_page_is_not_parsed(self):
        before = netlist.as_dict(netlist.build(self.project, self.root))
        self.copy_page()
        self.add_board("mux", BREAKOUT_PAGE)
        after = netlist.as_dict(netlist.build(self.project, self.root))
        self.assertEqual(after, before)

    def copy_page(self) -> None:
        dest = self.root / BREAKOUT_PAGE
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(
            (REPO_ROOT / BREAKOUT_PAGE).read_text(encoding="utf-8"), encoding="utf-8"
        )

    def test_a_pin_numbers_part_on_a_breakout_page_is_a_fail_line(self):
        # The join accepts the page (it has a physical layout); the pin_numbers
        # mapping needs a GPIO pin table, which a breakout page does not have.
        self.copy_page()
        with self.toml.open("a", encoding="utf-8") as f:
            f.write(
                '\n[parts.spare]\nname = "Spare XIAO"\nkind = "mcu"\n'
                'part = "seeed-xiao-rp2350-socket"\nref = "U2"\n'
                f'board = "{BREAKOUT_PAGE}"\n'
            )
        msg = r"adafruit-tca9548a\.md: no pin-mapping table"
        with self.assertRaisesRegex(KicadError, msg):
            netlist.build(self.project, self.root)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(netlist.main([str(self.project)], self.root), 1)
        self.assertRegex(out.getvalue(), rf"(?m)^FAIL: .*{msg}")


if __name__ == "__main__":
    unittest.main()
