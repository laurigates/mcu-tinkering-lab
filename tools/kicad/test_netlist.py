"""Tests for the carrier netlist built from the hardware join (spec §3.2, §5).

The balancebot netlist must equal `testdata/balancebot-carrier.net.json`,
which is `parity.normalise` of KiCad's own netlist export of the #749
schematic (`testdata/balancebot-carrier.xml`). Mutations run on a temp copy of
the project, its board page and the parts registry; the committed files are
only read.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import json
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from kicad import netlist
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

PROJECT_REL = Path("packages/robotics/balancebot")
PROJECT = REPO_ROOT / PROJECT_REL
TESTDATA = Path(__file__).resolve().parent / "testdata"
FIXTURE = TESTDATA / "balancebot-carrier.net.json"
EXPORT = TESTDATA / "balancebot-carrier.xml"


def fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


class BalancebotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT)

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
            self.assertEqual(netlist.main([str(PROJECT)]), 0)
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
        nl = netlist.build(PROJECT)
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
    """A temp copy of the repo pieces the join reads; each edit must change or fail the build."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        shutil.copytree(PROJECT / "src", self.root / PROJECT_REL / "src")
        shutil.copy2(
            PROJECT / "hardware.toml", self.root / PROJECT_REL / "hardware.toml"
        )
        for rel in ("docs/reference/boards", "docs/reference/parts"):
            shutil.copytree(REPO_ROOT / rel, self.root / rel)
        self.project = self.root / PROJECT_REL
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
        self.edit(
            self.toml,
            '[[nets]]\nrole = "PIN_IMU_INT"\n',
            '[[nets]]\nrole = "PIN_IMU_INT"\nto = "imu.SDA"\n\n[[nets]]\nrole = "PIN_IMU_INT"\n',
        )
        self.fails(r"J3\.4 is on net 'IMU_INT' and net 'IMU_SDA'")

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


if __name__ == "__main__":
    unittest.main()
