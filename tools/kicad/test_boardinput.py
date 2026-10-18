"""Tests for the board input the pcbnew scripts read (ADR-027).

Built from the frozen balancebot inputs (`kicad.testfixture`: hardware.toml,
pin_config.h and board.toml as they were for #749), not the live project;
board.toml mutations run on a deep copy of its parsed data. KiCad's own
netlist export of the #749 schematic (`testdata/balancebot-carrier.xml`) is
the oracle for the net names, the `unconnected-(...)` ones included.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import copy
import json
import os
import unittest
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from kicad import (
    boardcheck,
    boardinput,
    library,
    netlist,
    paths,
    schematic,
    testfixture,
)
from kicad.errors import KicadError

ROOT, PROJECT = testfixture.shared()
U1_UUID = "4fca1c60-d738-5efa-89ff-a0502ae7fc74"  # uuid5(ns, "fp/U1")
EXPORT = Path(__file__).resolve().parent / "testdata/balancebot-carrier.xml"
KEYS = {
    "pcb",
    "out_dir",
    "origin",
    "size",
    "components",
    "nets",
    "silk",
    "classes",
    "signal",
    "pour",
    "tracks",
    "keepouts",
    "cutouts",
}
COMPONENT_KEYS = {
    "ref",
    "value",
    "footprint",
    "fp_dir",
    "path",
    "uuid",
    "fields",
    "at",
}


class BalancebotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = paths.load_board_config(PROJECT)
        cls.nl = netlist.build(PROJECT, ROOT)
        cls.ns = schematic.namespace(schematic.project_key(PROJECT, ROOT))
        cls.kicad_dir = paths.carrier_dir(PROJECT) / cls.cfg.name
        cls.spec = boardinput.build(PROJECT, ROOT)
        cls.export = [n.get("name") for n in ET.parse(EXPORT).getroot().iter("net")]

    def assemble(self, data):
        cfg = paths.BoardConfig(self.cfg.path, self.cfg.name, self.cfg.paper, data)
        return boardinput.assemble(cfg, self.nl, self.ns, self.kicad_dir)

    def test_shape(self):
        self.assertEqual(set(self.spec), KEYS)  # no freerouting_jar: main adds it
        self.assertEqual(json.loads(json.dumps(self.spec)), self.spec)
        self.assertEqual(
            Path(self.spec["pcb"]),
            (self.kicad_dir / "balancebot-carrier.kicad_pcb").resolve(),
        )
        self.assertEqual(
            Path(self.spec["out_dir"]), (self.kicad_dir / "output").resolve()
        )
        self.assertEqual(
            (self.spec["origin"], self.spec["size"]), ([100.0, 100.0], [90.0, 62.0])
        )

    def test_components(self):
        comps = self.spec["components"]
        self.assertEqual([c["ref"] for c in comps], [c.ref for c in self.nl.comps])
        placement = self.cfg.data["placement"]
        for c, want in zip(comps, self.nl.comps, strict=True):
            with self.subTest(ref=c["ref"]):
                self.assertEqual(set(c), COMPONENT_KEYS)
                self.assertEqual(
                    (c["value"], c["footprint"], c["fields"]),
                    (want.value, want.footprint, [list(f) for f in want.fields]),
                )
                self.assertEqual(c["at"], placement[c["ref"]])
                self.assertEqual(c["path"], boardcheck.symbol_path(self.ns, c["ref"]))
                fp_dir = Path(c["fp_dir"])
                self.assertTrue(fp_dir.is_relative_to(library.VENDORED))
                name = c["footprint"].split(":", 1)[1]
                self.assertTrue((fp_dir / f"{name}.kicad_mod").is_file())
        u1 = next(c for c in comps if c["ref"] == "U1")
        self.assertEqual(u1["path"], "/5c733e52-0b61-5793-9755-4792b125be5a")
        r1 = next(c for c in comps if c["ref"] == "R1")
        self.assertEqual(r1["fields"], [["LCSC", "C17414"]])

    def test_footprint_uuids(self):
        # build_board.py sets these, so a rebuild orders the board, and the DSN
        # export that follows its footprint order, the same way (#763).
        comps = self.spec["components"]
        for c in comps:
            with self.subTest(ref=c["ref"]):
                self.assertEqual(c["uuid"], str(uuid.uuid5(self.ns, f"fp/{c['ref']}")))
                self.assertNotEqual(f"/{c['uuid']}", c["path"])
        self.assertEqual(len({c["uuid"] for c in comps}), len(comps))
        u1 = next(c for c in comps if c["ref"] == "U1")
        self.assertEqual(u1["uuid"], U1_UUID)

    def test_nets_are_kicads(self):
        names = [n["name"] for n in self.spec["nets"]]
        self.assertEqual(names, self.export)  # KiCad's names and order
        unconnected = {
            n["name"]: n["nodes"]
            for n in self.spec["nets"]
            if n["name"].startswith("unconnected-")
        }
        self.assertEqual(len(unconnected), 6)
        self.assertEqual(unconnected["unconnected-(A1-~{FLT}-Pad2)"], [["A1", "2"]])
        for n in self.spec["nets"]:
            if n["name"] in self.nl.nets:
                self.assertEqual(n["nodes"], [list(x) for x in self.nl.nets[n["name"]]])

    def test_class_expansion_order(self):
        motors = [f"MOT{s}_{c}" for s in "LR" for c in ("A1", "A2", "B1", "B2")]
        self.assertEqual(
            [(c["name"], c["nets"]) for c in self.spec["classes"]],
            [
                ("power", ["VBAT", "GND", *motors]),
                ("logic_power", ["+3V3", "BUCK_5V", "XIAO_5V"]),
            ],
        )
        self.assertEqual(self.spec["classes"][0]["via_um"], [1000, 500])
        self.assertEqual(self.spec["signal"]["via_um"], [600, 300])
        # A single-net [pour] is one region over the whole board (rect None).
        self.assertEqual(self.spec["pour"]["regions"], [{"net": "GND", "rect": None}])
        self.assertNotIn("net", self.spec["pour"])
        self.assertEqual(self.spec["pour"]["layers"], ["F.Cu", "B.Cu"])
        self.assertEqual(self.spec["keepouts"], [])
        self.assertEqual(self.spec["cutouts"], [])
        self.assertEqual(self.spec["silk"]["cut_lines"], [])

    def test_placement_missing(self):
        data = copy.deepcopy(self.cfg.data)
        del data["placement"]["U1"]
        with self.assertRaisesRegex(KicadError, r"\[placement\] has no entry for U1"):
            self.assemble(data)

    def test_placement_extra(self):
        data = copy.deepcopy(self.cfg.data)
        data["placement"]["R9"] = [1.0, 2.0, 0]
        with self.assertRaisesRegex(
            KicadError, r"\[placement\] R9 is not in the netlist"
        ):
            self.assemble(data)

    def test_bad_class_pattern(self):
        data = copy.deepcopy(self.cfg.data)
        data["net_classes"][1]["nets"].append("VMOT*")
        with self.assertRaisesRegex(KicadError, "'VMOT\\*' matches no net"):
            self.assemble(data)

    def test_keepouts_cut_lines_and_pour_regions(self):
        data = copy.deepcopy(self.cfg.data)
        band = {"name": "cut band", "rect": [40.0, 0.0, 43.0, 62.0]}
        data["keepouts"] = [band]
        hole = {"name": "relief", "center": [41.5, 61.0], "diameter": 1.0}
        data["cutouts"] = [hole]
        data["silk"]["cut_lines"] = [[41.5, 0.5, 41.5, 61.5]]
        del data["pour"]["net"]
        data["pour"]["regions"] = [
            {"net": "GND", "rect": [0, 0, 40, 62]},
            {"net": "+3V3", "rect": [43, 0, 90, 62]},
        ]
        spec = self.assemble(data)
        self.assertEqual(spec["keepouts"], [band])
        self.assertEqual(spec["cutouts"], [hole])
        self.assertEqual(spec["silk"]["cut_lines"], [[41.5, 0.5, 41.5, 61.5]])
        self.assertEqual(
            spec["pour"]["regions"],
            [
                {"net": "GND", "rect": [0, 0, 40, 62]},
                {"net": "+3V3", "rect": [43, 0, 90, 62]},
            ],
        )

    def test_pour_region_layers_and_priority_pass_through(self):
        data = copy.deepcopy(self.cfg.data)
        del data["pour"]["net"]
        data["pour"]["regions"] = [
            {"net": "GND", "rect": [0, 0, 90, 62], "priority": 0},
            {
                "net": "+3V3",
                "rect": [10, 10, 40, 40],
                "layers": ["F.Cu"],
                "priority": 1,
            },
        ]
        self.assertEqual(
            self.assemble(data)["pour"]["regions"],
            [
                {"net": "GND", "rect": [0, 0, 90, 62], "priority": 0},
                {
                    "net": "+3V3",
                    "rect": [10, 10, 40, 40],
                    "layers": ["F.Cu"],
                    "priority": 1,
                },
            ],
        )

    def test_tracks_pass_through(self):
        data = copy.deepcopy(self.cfg.data)
        self.assertEqual(self.assemble(data)["tracks"], [])
        trunk = {
            "name": "trunk",
            "net": "GND",
            "layer": "F.Cu",
            "width_mm": 1.5,
            "points": [[5, 5], [5, 30]],
        }
        data["tracks"] = [trunk]
        self.assertEqual(self.assemble(data)["tracks"], [trunk])

    def test_silk_defaults(self):
        data = copy.deepcopy(self.cfg.data)
        del data["silk"]
        self.assertEqual(self.assemble(data)["silk"], boardinput.SILK_DEFAULTS)

    def test_input_path(self):
        self.assertEqual(
            boardinput.input_path(PROJECT),
            PROJECT / "hardware/carrier/output/board-input.json",
        )

    def test_freerouting_jar(self):
        with mock.patch.dict(os.environ, {"FREEROUTING_JAR": "/opt/fr.jar"}):
            self.assertEqual(boardinput.freerouting_jar(), "/opt/fr.jar")
        with mock.patch.dict(os.environ, {}, clear=True):
            os.environ["HOME"] = "/home/x"
            self.assertEqual(
                boardinput.freerouting_jar(),
                "/home/x/.cache/freerouting/freerouting-2.5.0.jar",
            )


if __name__ == "__main__":
    unittest.main()
