"""Tests for the check stamp (spec §5.5).

Synthetic stamps and files only: the committed stamp is refreshed by a local
KiCad run, so CI's `python3 -m kicad.stamp` step checks it, not this suite
(spec §9).

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path

from kicad import paths, stamp

SCH = b"(kicad_sch)\n"
PCB = b"(kicad_pcb)\n"
NETLIST = {
    "components": {
        "R1": {
            "footprint": "Resistor_SMD:R_0805",
            "in_bom": True,
            "lcsc": "C17414",
            "lib_id": "Device:R",
            "value": "10k",
        }
    },
    "nets": {"A": [["R1", "1"]]},
    "unconnected": {"unconnected-(R1-Pad2)": ["R1", "2"]},
}


class CompareTests(unittest.TestCase):
    def stamp(self) -> dict:
        return json.loads(stamp.render(SCH, PCB, copy.deepcopy(NETLIST), "10.0.6"))

    def test_clean(self):
        self.assertEqual(stamp.compare(self.stamp(), SCH, PCB, NETLIST), [])

    def test_format(self):
        text = stamp.render(SCH, PCB, NETLIST, "10.0.6")
        self.assertTrue(text.endswith("}\n"))
        self.assertTrue(text.startswith('{\n "board_sha256": "'))
        self.assertEqual(list(json.loads(text)), sorted(stamp.KEYS))

    def test_schematic_sha(self):
        errors = stamp.compare(self.stamp(), SCH + b" ", PCB, NETLIST)
        self.assertEqual(
            errors, ["schematic_sha256 differs from the committed .kicad_sch"]
        )

    def test_board_sha(self):
        errors = stamp.compare(self.stamp(), SCH, PCB.replace(b"pcb", b"pcB"), NETLIST)
        self.assertEqual(errors, ["board_sha256 differs from the committed .kicad_pcb"])

    def test_net_member(self):
        want = copy.deepcopy(NETLIST)
        want["nets"]["A"] = [["R1", "1"], ["R1", "3"]]
        errors = stamp.compare(self.stamp(), SCH, PCB, want)
        self.assertEqual(errors, ["netlist: net A: join R1.1 R1.3, stamp R1.1"])

    def test_unconnected_name(self):
        want = copy.deepcopy(NETLIST)
        want["unconnected"] = {"unconnected-(R1-B-Pad2)": ["R1", "2"]}
        errors = stamp.compare(self.stamp(), SCH, PCB, want)
        self.assertEqual(len(errors), 1)
        self.assertIn("no-connects differ", errors[0])

    def test_kicad_version_is_recorded_not_compared(self):
        s = self.stamp()
        s["kicad_cli_version"] = "11.0.0"
        self.assertEqual(stamp.compare(s, SCH, PCB, NETLIST), [])

    def test_malformed(self):
        s = self.stamp()
        del s["netlist"]
        self.assertEqual(
            stamp.compare(s, SCH, PCB, NETLIST), ["stamp lacks ['netlist']"]
        )


class CliTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.project = Path(self.tmp.name) / "packages/demo"
        carrier = paths.carrier_dir(self.project)
        self.kicad_dir = carrier / "demo-carrier"
        self.kicad_dir.mkdir(parents=True)
        (carrier / "board.toml").write_text('[project]\nname = "demo-carrier"\n')
        (self.kicad_dir / "demo-carrier.kicad_sch").write_bytes(SCH)
        (self.kicad_dir / "demo-carrier.kicad_pcb").write_bytes(PCB)

    def run_main(self) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = stamp.main([str(self.project)], repo_root=Path(self.tmp.name))
        return code, out.getvalue()

    def test_missing_stamp_names_carrier_check(self):
        code, out = self.run_main()
        self.assertEqual(code, 1)
        self.assertIn("demo-carrier.check.json: missing", out)
        self.assertIn("just demo::carrier-check", out)
        self.assertIn("do not edit it or the .kicad_pcb by hand", out)

    def test_stamp_that_is_not_json(self):
        (self.kicad_dir / "demo-carrier.check.json").write_text("{")
        code, out = self.run_main()
        self.assertEqual(code, 1)
        self.assertIn("not JSON", out)

    def test_paths(self):
        s, sch, pcb = stamp.files(self.project)
        self.assertEqual(
            (s.name, sch.name, pcb.name),
            (
                "demo-carrier.check.json",
                "demo-carrier.kicad_sch",
                "demo-carrier.kicad_pcb",
            ),
        )


if __name__ == "__main__":
    unittest.main()
