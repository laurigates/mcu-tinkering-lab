"""Tests for netlist parity (spec §5 item 1), with no KiCad.

`testdata/balancebot-carrier.xml` is `kicad-cli sch export netlist --format
kicadxml` (KiCad 10.0.6) of #749's committed balancebot-carrier.kicad_sch, with
`<source>` made repo-relative and the export `<date>` blanked. The `kicad.parity`
CLI runs that export itself; here it is replaced by the fixture.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from kicad import netlist, parity
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

PROJECT = REPO_ROOT / "packages/robotics/balancebot"
TESTDATA = Path(__file__).resolve().parent / "testdata"
EXPORT = TESTDATA / "balancebot-carrier.xml"


def export() -> ET.Element:
    return ET.parse(EXPORT).getroot()


def net(root: ET.Element, name: str) -> ET.Element:
    (found,) = [n for n in root.find("nets") if n.get("name") == name]
    return found


def comp(root: ET.Element, ref: str) -> ET.Element:
    (found,) = [c for c in root.iter("comp") if c.get("ref") == ref]
    return found


class CompareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT)

    def test_the_749_export_matches_the_join(self):
        self.assertEqual(parity.compare(self.nl, export()), [])

    def test_normalised_export_is_the_json_fixture(self):
        want = json.loads((TESTDATA / "balancebot-carrier.net.json").read_text())
        self.assertEqual(parity.normalise(export()), want)

    def test_swapped_step_and_dir(self):
        """E9: STEP_L and DIR_L swapped at the XIAO."""
        root = export()
        step = [n for n in net(root, "STEP_L") if n.get("ref") == "U1"][0]
        dirn = [n for n in net(root, "DIR_L") if n.get("ref") == "U1"][0]
        a, b = step.get("pin"), dirn.get("pin")
        step.set("pin", b)
        dirn.set("pin", a)
        errors = parity.compare(self.nl, root)
        self.assertEqual(len(errors), 2, errors)
        self.assertTrue(errors[0].startswith("net DIR_L: join A1.16 U1.10, schematic"))

    def test_renamed_net(self):
        root = export()
        net(root, "IMU_SDA").set("name", "IMU_SDA2")
        errors = parity.compare(self.nl, root)
        self.assertEqual(
            errors,
            [
                "net IMU_SDA: in the join (J3.4 U1.5), not in the schematic",
                "net IMU_SDA2: in the schematic (J3.4 U1.5), not in the join",
            ],
        )

    def test_value_change(self):
        root = export()
        comp(root, "R2").find("value").text = "100K"
        self.assertEqual(
            parity.compare(self.nl, root),
            ["component R2 value: join '100k', schematic '100K'"],
        )

    def test_lcsc_and_bom_changes(self):
        root = export()
        for f in comp(root, "R1").iter("field"):
            if f.get("name") == "LCSC":
                f.text = "C1"
        hole = comp(root, "H1")
        for p in list(hole.iter("property")):
            if p.get("name") == "exclude_from_bom":
                hole.remove(p)
        self.assertEqual(
            parity.compare(self.nl, root),
            [
                "component H1 in_bom: join False, schematic True",
                "component R1 lcsc: join 'C17414', schematic 'C1'",
            ],
        )

    def test_no_connect_change(self):
        root = export()
        nets = root.find("nets")
        nets.remove(net(root, "unconnected-(J3-Pin_6-Pad6)"))
        errors = parity.compare(self.nl, root)
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("unconnected-(J3-Pin_6-Pad6)", errors[0])

    def test_no_connect_rename(self):
        root = export()
        net(root, "unconnected-(U1-D1-Pad2)").set("name", "unconnected-(U1-Pad2)")
        self.assertEqual(len(parity.compare(self.nl, root)), 1)

    def test_missing_and_extra_component(self):
        root = export()
        comps = root.find("components")
        comps.remove(comp(root, "H4"))
        extra = ET.SubElement(comps, "comp", ref="TP1")
        ET.SubElement(extra, "libsource", lib="Connector", part="TestPoint")
        self.assertEqual(
            parity.compare(self.nl, root),
            [
                "component H4: in the join, not in the schematic",
                "component TP1: in the schematic, not in the join",
            ],
        )

    def test_power_flags_are_not_components_or_nodes(self):
        root = export()
        flag = ET.SubElement(root.find("components"), "comp", ref="#FLG01")
        ET.SubElement(flag, "libsource", lib="power", part="PWR_FLAG")
        ET.SubElement(net(root, "GND"), "node", ref="#FLG01", pin="1")
        self.assertEqual(parity.compare(self.nl, root), [])

    def test_order_is_not_compared(self):
        root = export()
        gnd = net(root, "GND")
        nodes = list(gnd)
        for n in nodes:
            gnd.remove(n)
        for n in reversed(nodes):
            gnd.append(n)
        self.assertEqual(parity.compare(self.nl, root), [])

    def test_unconnected_net_with_two_nodes_is_rejected(self):
        root = export()
        ET.SubElement(
            net(root, "unconnected-(A1-~{FLT}-Pad2)"), "node", ref="A2", pin="2"
        )
        with self.assertRaisesRegex(KicadError, "has 2 nodes, expected 1"):
            parity.normalise(root)


class CliTests(unittest.TestCase):
    def run_main(self, root: ET.Element) -> tuple[int, str]:
        out = io.StringIO()
        with (
            mock.patch.object(parity, "export", return_value=root),
            contextlib.redirect_stdout(out),
        ):
            code = parity.main([str(PROJECT)])
        return code, out.getvalue()

    def test_clean(self):
        code, out = self.run_main(export())
        self.assertEqual(code, 0, out)
        self.assertIn("0 differences (19 components, 22 nets, 6 no-connects)", out)

    def test_difference_fails(self):
        root = export()
        net(root, "IMU_SDA").set("name", "IMU_SDA2")
        code, out = self.run_main(root)
        self.assertEqual(code, 1)
        self.assertIn("PARITY FAIL", out)
        self.assertIn("FAIL: net IMU_SDA: in the join", out)

    def test_schematic_path(self):
        self.assertEqual(
            parity.schematic_path(PROJECT),
            PROJECT
            / "hardware/carrier/balancebot-carrier/balancebot-carrier.kicad_sch",
        )

    def test_missing_schematic(self):
        with self.assertRaisesRegex(KicadError, "no schematic"):
            parity.export(PROJECT / "no-such.kicad_sch")


if __name__ == "__main__":
    unittest.main()
