"""Tests for the schematic writer (spec §4, §11.1).

The geometry checks parse the written text back: every wire runs 2.54 mm from a
pin point to its label, and no point of the sheet joins two nets. They share
the writer's pin transform, so a sign error there would pass them; the check
stamp (`kicad.stamp`) is what catches that, because KiCad itself must read the
committed schematic and produce the join's netlist.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import dataclasses
import io
import json
import re
import shutil
import tempfile
import unittest
import uuid
from pathlib import Path

from kicad import netlist, paths, schematic, sexpr, symbols
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

PROJECT_REL = Path("packages/robotics/balancebot")
PROJECT = REPO_ROOT / PROJECT_REL
KEY = "packages/robotics/balancebot/hardware/carrier"
COMMITTED = PROJECT / "hardware/carrier/balancebot-carrier/balancebot-carrier.kicad_sch"
_XY = r"(-?[\d.]+) (-?[\d.]+)"


def copy_repo(root: Path) -> Path:
    """The repo pieces `kicad.schematic` reads, under `root`; returns the project.

    The vendored library is a symlink, so the generated library-table paths
    have the committed depth.
    """
    project = root / PROJECT_REL
    shutil.copytree(PROJECT / "src", project / "src")
    shutil.copy2(PROJECT / "hardware.toml", project / "hardware.toml")
    carrier = paths.carrier_dir(PROJECT)
    shutil.copytree(
        carrier,
        paths.carrier_dir(project),
        ignore=shutil.ignore_patterns("output", "scripts", "*.kicad_prl"),
    )
    for rel in ("docs/reference/boards", "docs/reference/parts"):
        shutil.copytree(REPO_ROOT / rel, root / rel)
    (root / "tools/kicad").mkdir(parents=True)
    (root / "tools/kicad/lib").symlink_to(REPO_ROOT / "tools/kicad/lib")
    return project


def parse(text: str) -> dict:
    """Wires, labels, no-connects and symbols of a written schematic."""
    wires = [
        ((float(a), float(b)), (float(c), float(d)))
        for a, b, c, d in re.findall(rf"\(pts\n\t\t\t\(xy {_XY}\) \(xy {_XY}\)", text)
    ]
    labels = [
        (sexpr.unquote(n), (float(x), float(y)), int(a))
        for n, x, y, a in re.findall(
            rf'^\t\(global_label "((?:[^"\\]|\\.)*)"\n\t\t\(shape bidirectional\)\n'
            rf"\t\t\(at {_XY} (\d+)\)",
            text,
            re.M,
        )
    ]
    ncs = [
        (float(x), float(y))
        for x, y in re.findall(rf"^\t\(no_connect\n\t\t\(at {_XY}\)", text, re.M)
    ]
    syms = []
    for _, block in sexpr.top_level_blocks(text, "\t"):
        if sexpr.head(block)[0] != "symbol":
            continue
        lib_id = sexpr.head(sexpr.child(block, "lib_id"))[1]
        at = [float(v) for v in sexpr.atoms(sexpr.child(block, "at"))[:2]]
        ref = re.search(r'\(property "Reference" "([^"]*)"', block).group(1)
        syms.append((ref, lib_id, (at[0], at[1]), block))
    return {"wires": wires, "labels": labels, "ncs": ncs, "symbols": syms}


class BalancebotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT)
        cls.text = schematic.render(cls.nl, "balancebot-carrier", KEY)
        cls.sheet = parse(cls.text)

    def test_deterministic(self):
        self.assertEqual(
            schematic.render(self.nl, "balancebot-carrier", KEY), self.text
        )

    def test_committed_schematic_is_current(self):
        files = schematic.outputs(PROJECT)
        for path, text in files.items():
            with self.subTest(path=path.name):
                self.assertEqual(
                    path.read_text(encoding="utf-8"),
                    text,
                    "run `just balancebot::carrier-schematic`",
                )

    def test_counts(self):
        syms = self.sheet["symbols"]
        flags = [s for s in syms if s[1] == schematic.FLAG]
        self.assertEqual((len(syms), len(flags)), (19 + 2, 2))
        self.assertEqual([s[0] for s in flags], ["#FLG01", "#FLG02"])
        connected = sum(len(nodes) for nodes in self.nl.nets.values())
        self.assertEqual(len(self.sheet["labels"]), connected + len(flags))
        self.assertEqual(len(self.sheet["wires"]), connected + len(flags))
        self.assertEqual(len(self.sheet["ncs"]), 6)

    def test_power_flags_on_gnd_and_vbat(self):
        text = self.text
        for net in ("GND", "VBAT"):
            uid = str(uuid.uuid5(schematic.namespace(KEY), f"flag/{net}"))
            self.assertIn(f'(uuid "{uid}")', text)

    def test_every_wire_is_a_stub_from_a_pin_to_its_label(self):
        pin_points = set()
        for ref, lib_id, at, _ in self.sheet["symbols"]:
            for p in symbols.pins(symbols.load(lib_id)):
                pin_points.add(schematic.pin_point(at, p))
        labels = {at for _, at, _ in self.sheet["labels"]}
        for a, b in self.sheet["wires"]:
            self.assertIn(a, pin_points)
            self.assertIn(b, labels)
            self.assertAlmostEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), schematic.STUB)
            self.assertTrue(a[0] == b[0] or a[1] == b[1])
        for nc in self.sheet["ncs"]:
            self.assertIn(nc, pin_points)

    def test_no_point_joins_two_nets(self):
        net_at: dict[tuple[float, float], set[str]] = {}
        label_net = {at: name for name, at, _ in self.sheet["labels"]}
        for a, b in self.sheet["wires"]:
            for point in (a, b):
                net_at.setdefault(point, set()).add(label_net[b])
        for i, point in enumerate(self.sheet["ncs"]):
            net_at.setdefault(point, set()).add(f"no-connect {i}")
        joined = {p: n for p, n in net_at.items() if len(n) > 1}
        self.assertEqual(joined, {})

    def test_labels_name_the_netlist_nets(self):
        names = {name for name, _, _ in self.sheet["labels"]}
        self.assertEqual(names, set(self.nl.nets))

    def test_uuids_unique_and_golden(self):
        uids = re.findall(r'\(uuid "([0-9a-f-]+)"\)', self.text)
        self.assertEqual(len(uids), len(set(uids)))
        ns = schematic.namespace(KEY)
        self.assertEqual(str(ns), "d5cf8564-8090-54d5-9600-a71cba86fc98")
        self.assertEqual(
            str(uuid.uuid5(ns, "sym/U1")), "5c733e52-0b61-5793-9755-4792b125be5a"
        )
        self.assertEqual(schematic.project_key(PROJECT), KEY)

    def test_instance_flags_follow_the_library(self):
        h1 = next(s for s in self.sheet["symbols"] if s[0] == "H1")[3]
        self.assertIn("\t\t(in_bom no)\n\t\t(on_board yes)\n\t\t(in_pos_files yes)", h1)
        r1 = next(s for s in self.sheet["symbols"] if s[0] == "R1")[3]
        self.assertIn('(property "LCSC" "C17414"', r1)

    def test_lib_symbols_embed_each_used_symbol_once(self):
        lib = sexpr.child(self.text, "lib_symbols")
        names = [sexpr.head(c)[1] for c in sexpr.children(lib)]
        self.assertEqual(
            names, sorted({c.lib_id for c in self.nl.comps} | {schematic.FLAG})
        )

    def test_quotes_are_escaped(self):
        comps = list(self.nl.comps)
        comps[0] = dataclasses.replace(comps[0], value='4"7 (x)')
        nl = dataclasses.replace(self.nl, comps=tuple(comps))
        text = schematic.render(nl, 'demo "q"', KEY)
        sheet = parse(text)
        self.assertEqual(len(sheet["symbols"]), 21)
        block = next(s for s in sheet["symbols"] if s[0] == comps[0].ref)[3]
        values = [
            sexpr.unquote(m)
            for m in re.findall(r'\(property "Value" "((?:[^"\\]|\\.)*)"', block)
        ]
        self.assertEqual(values, ['4"7 (x)'])
        self.assertIn('(title "demo \\"q\\"")', text)
        self.assertEqual(len(sexpr.spans(text)), 1)

    def test_numbers(self):
        self.assertEqual(schematic.fmt(-0.00001), "0")
        self.assertEqual(schematic.fmt(25.4), "25.4")
        self.assertEqual(schematic.fmt(3.0), "3")
        self.assertEqual(schematic.snap(1.3), 2.54)

    def test_overflow_moves_to_a_larger_paper(self):
        r = schematic._item(
            "R1", "Device:R", "1k", "", (), symbols.load("Device:R"), "sym/R1", "R1"
        )
        self.assertIsNotNone(schematic._pack([[r] * 3], schematic.PAPERS["A4"]))
        self.assertIsNone(schematic._pack([[r] * 200], schematic.PAPERS["A4"]))
        text = schematic.render(self.nl, "balancebot-carrier", KEY, paper="A4")
        self.assertIn('(paper "A3")', text)  # 21 symbols overflow A4
        with self.assertRaisesRegex(KicadError, "paper 'B5'"):
            schematic.render(self.nl, "x", KEY, paper="B5")

    def test_stacked_pins_on_different_nets_raise(self):
        pin = symbols.pins(symbols.load("Device:R"))[0]
        twin = dataclasses.replace(pin, number="9")
        item = schematic._Item(
            "R9",
            "Device:R",
            "1k",
            "",
            (),
            "",
            (pin, twin),
            (0, 0, 0, 0),
            "sym/R9",
            "R9",
        )
        with self.assertRaisesRegex(KicadError, "stacked"):
            schematic._check_stacked(item, {("R9", pin.number): "A", ("R9", "9"): "B"})
        schematic._check_stacked(item, {("R9", pin.number): "A", ("R9", "9"): "A"})


class CliTests(unittest.TestCase):
    """`--check` on a temp copy of the repo pieces it reads."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.project = copy_repo(self.root)
        self.kicad_dir = paths.carrier_dir(self.project) / "balancebot-carrier"

    def run_main(self, *argv: str) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = schematic.main([*argv, str(self.project)], repo_root=self.root)
        return code, out.getvalue()

    def edit(self, path: Path, old: str, new: str) -> None:
        text = path.read_text(encoding="utf-8")
        self.assertIn(old, text)
        path.write_text(text.replace(old, new, 1), encoding="utf-8")

    def test_unedited_copy_is_current(self):
        code, out = self.run_main("--check")
        self.assertEqual(code, 0, out)
        self.assertIn("up to date", out)

    def test_edited_label_is_stale(self):
        self.edit(
            self.kicad_dir / "balancebot-carrier.kicad_sch",
            '(global_label "STEP_L"',
            '(global_label "DIR_L"',
        )
        code, out = self.run_main("--check")
        self.assertEqual(code, 1)
        self.assertIn("balancebot-carrier.kicad_sch", out)
        self.assertIn("carrier-schematic", out)

    def test_edited_lib_table_is_stale(self):
        self.edit(
            self.kicad_dir / "fp-lib-table", "../../../../../../", "../../../../../"
        )
        code, out = self.run_main("--check")
        self.assertEqual(code, 1)
        self.assertIn("fp-lib-table", out)

    def test_moved_pin_is_stale_then_regenerates(self):
        self.edit(
            self.project / "src/pin_config.h",
            "#define PIN_DIR_LEFT 4 ",
            "#define PIN_DIR_LEFT 27 ",
        )
        self.assertEqual(self.run_main("--check")[0], 1)
        code, out = self.run_main()
        self.assertEqual(code, 0, out)
        self.assertIn("wrote", out)
        self.assertEqual(self.run_main("--check")[0], 0)
        self.assertEqual(self.run_main()[1].splitlines()[-1], "up to date")

    def test_gui_rewritten_top_level_sheets(self):
        pro = self.kicad_dir / "balancebot-carrier.kicad_pro"
        data = json.loads(pro.read_text(encoding="utf-8"))
        data["schematic"]["top_level_sheets"] = []
        pro.write_text(json.dumps(data), encoding="utf-8")
        code, out = self.run_main("--check")
        self.assertEqual(code, 1)
        self.assertIn("top_level_sheets[0].filename is None", out)
        pro.unlink()
        code, out = self.run_main("--check")
        self.assertEqual(code, 1)
        self.assertIn("kicad_pro: missing", out)


if __name__ == "__main__":
    unittest.main()
