"""Tests for the generated project library tables (spec §3.3).

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import re
import shutil
import tempfile
import unittest
from pathlib import Path

from kicad import libtables, library, netlist, schematic
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

PROJECT = REPO_ROOT / "packages/robotics/balancebot"
_ENTRY = re.compile(
    r'^  \(lib \(name "([^"]+)"\)\(type "KiCad"\)\(uri "([^"]+)"\)', re.M
)


def entries(table: str) -> list[tuple[str, str]]:
    return _ENTRY.findall(table)


class DepthTests(unittest.TestCase):
    """The `${KIPRJMOD}` path climbs exactly as far as the project sits deep."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.vendored = self.root / "tools/kicad/lib"
        shutil.copytree(library.VENDORED, self.vendored)

    def test_depths(self):
        for depth in (4, 6):
            with self.subTest(depth=depth):
                kicad_dir = self.root.joinpath(*(f"d{i}" for i in range(depth)))
                kicad_dir.mkdir(parents=True)
                up = "../" * depth
                sym = libtables.render_sym_lib_table(
                    kicad_dir,
                    ["Device:R", "Seeed_XIAO:XIAO-RP2350-Socket"],
                    self.vendored,
                )
                self.assertEqual(
                    entries(sym),
                    [
                        (
                            "Device",
                            f"${{KIPRJMOD}}/{up}tools/kicad/lib/kicad-stock/symbols/Device.kicad_sym",
                        ),
                        (
                            "Seeed_XIAO",
                            f"${{KIPRJMOD}}/{up}tools/kicad/lib/seeed-xiao/Seeed_XIAO.kicad_sym",
                        ),
                    ],
                )
                fp = libtables.render_fp_lib_table(
                    kicad_dir,
                    ["Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"],
                    self.vendored,
                )
                self.assertEqual(
                    entries(fp),
                    [
                        (
                            "Resistor_SMD",
                            f"${{KIPRJMOD}}/{up}tools/kicad/lib/kicad-stock/footprints/Resistor_SMD.pretty",
                        )
                    ],
                )

    def test_unvendored_entry_names_the_vendor_recipe(self):
        with self.assertRaisesRegex(KicadError, "just kicad::vendor"):
            libtables.render_sym_lib_table(
                self.root, ["Amplifier_Audio:LM386"], self.vendored
            )
        with self.assertRaisesRegex(KicadError, "just kicad::vendor"):
            libtables.render_fp_lib_table(self.root, ["Module:Nope"], self.vendored)

    def test_resolved_files_exist(self):
        kicad_dir = self.root / "a/b"
        kicad_dir.mkdir(parents=True)
        sym = libtables.render_sym_lib_table(
            kicad_dir, ["power:PWR_FLAG"], self.vendored
        )
        ((_, uri),) = entries(sym)
        self.assertTrue(
            (kicad_dir / uri.removeprefix("${KIPRJMOD}/")).resolve().is_file()
        )


class BalancebotTests(unittest.TestCase):
    def test_every_used_nickname_once_sorted(self):
        nl = netlist.build(PROJECT)
        files = schematic.outputs(PROJECT)
        sym = next(t for p, t in files.items() if p.name == "sym-lib-table")
        fp = next(t for p, t in files.items() if p.name == "fp-lib-table")
        want_sym = sorted({library.split_id(c.lib_id)[0] for c in nl.comps} | {"power"})
        want_fp = sorted({library.split_id(c.footprint)[0] for c in nl.comps})
        self.assertEqual([n for n, _ in entries(sym)], want_sym)
        self.assertEqual([n for n, _ in entries(fp)], want_fp)
        self.assertTrue(sym.startswith("(sym_lib_table\n  (version 7)\n"))
        self.assertTrue(fp.endswith(")\n"))

    def test_descriptions_name_source_and_licence(self):
        self.assertIn("CC-BY-SA 4.0", libtables.descr("Device"))
        self.assertIn("10.0.6", libtables.descr("Device"))
        self.assertIn("MIT", libtables.descr("Seeed_XIAO"))


if __name__ == "__main__":
    unittest.main()
