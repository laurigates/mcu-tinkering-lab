"""Tests for the KiCad-free board check and relink (ADR-027).

Synthetic boards only: the suite never reads the committed .kicad_pcb, which
only a local KiCad run can refresh (`carrier-check.yml` runs the CLI on it).
board.toml validation uses the frozen balancebot netlist and board.toml
(`kicad.testfixture`), not the live project.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import copy
import io
import tempfile
import tomllib
import unittest
import uuid
from pathlib import Path

from kicad import boardcheck, library, netlist, paths, schematic, testfixture
from kicad.errors import KicadError

NS = uuid.uuid5(uuid.NAMESPACE_URL, "mcu-tinkering-lab/kicad/test")
ROOT, PROJECT = testfixture.shared()


def comp(
    ref: str, lib_id: str, value: str, footprint: str, lcsc: str = ""
) -> netlist.Component:
    return netlist.Component(
        ref=ref,
        value=value,
        footprint=footprint,
        lib_id=lib_id,
        fields=(("LCSC", lcsc),) if lcsc else (),
        in_bom=True,
        on_board=True,
        part_key=ref.lower(),
        registry="test",
    )


NL = netlist.Netlist(
    project=Path("/repo/packages/demo"),
    comps=(
        comp("C1", "Device:C", "100nF", "Capacitor_SMD:C_0805", "C49678"),
        comp("J1", "Connector_Generic:Conn_01x02", "Pins", "Connector:J2"),
        comp("R1", "Device:R", "10k", "Resistor_SMD:R_0805", "C17414"),
    ),
    nets={"A": (("C1", "1"), ("R1", "1")), "B": (("J1", "1"), ("R1", "2"))},
    no_connect=(("C1", "2"), ("J1", "2")),
)
NC = {("C1", "2"): "unconnected-(C1-Pad2)", ("J1", "2"): "unconnected-(J1-Pin_2-Pad2)"}


def footprint(ref, fpid, value, lcsc, pads, path=None) -> str:
    path = path or boardcheck.symbol_path(NS, ref)
    lines = [
        f'\t(footprint "{fpid}"',
        '\t\t(layer "F.Cu")',
        '\t\t(uuid "00000000-0000-0000-0000-000000000000")',
        f'\t\t(property "Reference" "{ref}"',
        "\t\t\t(at 0 -1.65 0)",
        "\t\t)",
        f'\t\t(property "Value" "{value}"',
        "\t\t\t(at 0 1.65 0)",
        "\t\t)",
    ]
    if lcsc:
        lines += [f'\t\t(property "LCSC" "{lcsc}"', "\t\t\t(at 0 0 0)", "\t\t)"]
    lines.append(f'\t\t(path "{path}")')
    for num, net in pads:
        lines.append(f'\t\t(pad "{num}" smd rect')
        lines.append("\t\t\t(at 0 0)")
        if net:
            lines.append(f'\t\t\t(net "{net}")')
        lines.append("\t\t)")
    lines.append("\t)")
    return "\n".join(lines)


def board(**override) -> str:
    fps = {
        "C1": footprint(
            "C1",
            "Capacitor_SMD:C_0805",
            "100nF",
            "C49678",
            [("1", "A"), ("2", NC[("C1", "2")])],
        ),
        "J1": footprint(
            "J1",
            "Connector:J2",
            "Pins",
            "",
            [("1", "B"), ("2", NC[("J1", "2")]), ("", "")],
        ),
        "R1": footprint(
            "R1", "Resistor_SMD:R_0805", "10k", "C17414", [("1", "A"), ("2", "B")]
        ),
    }
    fps.update(override)
    body = [f for f in fps.values() if f]
    return '(kicad_pcb\n\t(version 20260206)\n\t(net "A")\n' + "\n".join(body) + "\n)\n"


class CheckTests(unittest.TestCase):
    def errors(self, text: str) -> list[str]:
        return boardcheck.check(text, NL, NS, module="demo")

    def fails(self, text: str, msg: str) -> None:
        errors = self.errors(text)
        self.assertEqual(len(errors), 1, errors)
        self.assertIn(msg, errors[0])
        self.assertIn("just demo::carrier-board", errors[0])
        self.assertIn("never hand-edit the .kicad_pcb", errors[0])

    def test_clean(self):
        self.assertEqual(self.errors(board()), [])

    def test_pad_net_swap(self):
        r1 = footprint(
            "R1", "Resistor_SMD:R_0805", "10k", "C17414", [("1", "A"), ("2", "A")]
        )
        self.fails(board(R1=r1), "R1.2: pad on 'A', netlist says 'B'")

    def test_nc_pad_without_its_net(self):
        c1 = footprint(
            "C1", "Capacitor_SMD:C_0805", "100nF", "C49678", [("1", "A"), ("2", "")]
        )
        self.fails(
            board(C1=c1),
            "C1.2: no-connect pad on '', KiCad names its net 'unconnected-(C1-Pad2)'",
        )

    def test_pad_on_a_net_the_netlist_lacks(self):
        r1 = footprint(
            "R1",
            "Resistor_SMD:R_0805",
            "10k",
            "C17414",
            [("1", "A"), ("2", "B"), ("3", "A")],
        )
        self.fails(board(R1=r1), "R1.3: pad on 'A', netlist has no net")

    def test_value(self):
        r1 = footprint(
            "R1", "Resistor_SMD:R_0805", "4k7", "C17414", [("1", "A"), ("2", "B")]
        )
        self.fails(board(R1=r1), "R1: value '4k7', netlist says '10k'")

    def test_lcsc(self):
        r1 = footprint(
            "R1", "Resistor_SMD:R_0805", "10k", "C1", [("1", "A"), ("2", "B")]
        )
        self.fails(board(R1=r1), "R1: LCSC 'C1', netlist says 'C17414'")

    def test_footprint(self):
        r1 = footprint(
            "R1", "Resistor_SMD:R_1206", "10k", "C17414", [("1", "A"), ("2", "B")]
        )
        self.fails(board(R1=r1), "R1: footprint 'Resistor_SMD:R_1206'")

    def test_path(self):
        r1 = footprint(
            "R1", "Resistor_SMD:R_0805", "10k", "C17414", [("1", "A"), ("2", "B")], "/x"
        )
        self.fails(board(R1=r1), "R1: path '/x'")

    def test_extra_footprint(self):
        r9 = footprint("R9", "Resistor_SMD:R_0805", "10k", "", [])
        self.fails(board(R9=r9), "R9: on the board, not in the netlist")

    def test_missing_footprint(self):
        self.fails(board(J1=""), "J1: in the netlist, not on the board")

    def test_duplicate_footprint(self):
        text = board()
        r1 = footprint(
            "R1", "Resistor_SMD:R_0805", "10k", "C17414", [("1", "A"), ("2", "B")]
        )
        self.fails(text.replace("\n)\n", "\n" + r1 + "\n)\n"), "R1: on the board twice")

    def test_parse(self):
        fp = boardcheck.parse_footprint(boardcheck.footprint_blocks(board())[1][2])
        self.assertEqual(
            (fp.fpid, fp.ref, fp.value, fp.lcsc),
            ("Connector:J2", "J1", "Pins", ""),
        )
        self.assertEqual(fp.pads, {"1": {"B"}, "2": {NC[("J1", "2")]}, "": {""}})


class RelinkTests(unittest.TestCase):
    def test_relink_only_changes_paths_and_is_idempotent(self):
        stale = board(
            R1=footprint(
                "R1",
                "Resistor_SMD:R_0805",
                "10k",
                "C17414",
                [("1", "A"), ("2", "B")],
                "/old-r1",
            ),
            C1=footprint(
                "C1",
                "Capacitor_SMD:C_0805",
                "100nF",
                "C49678",
                [("1", "A"), ("2", NC[("C1", "2")])],
                "/old-c1",
            ),
        )
        self.assertEqual(len(boardcheck.check(stale, NL, NS, module="demo")), 2)
        text, n = boardcheck.relink(stale, NS)
        self.assertEqual(n, 2)
        self.assertEqual(text, board())
        changed = [
            (a, b)
            for a, b in zip(stale.splitlines(), text.splitlines(), strict=True)
            if a != b
        ]
        self.assertEqual(len(changed), 2)
        self.assertTrue(all(b.startswith('\t\t(path "/') for _, b in changed))
        self.assertEqual(boardcheck.relink(text, NS), (text, 0))
        self.assertEqual(boardcheck.check(text, NL, NS, module="demo"), [])


class ConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nl = netlist.build(PROJECT, ROOT)
        cls.cfg = paths.load_board_config(PROJECT)

    def with_data(self, edit) -> paths.BoardConfig:
        data = copy.deepcopy(self.cfg.data)
        edit(data)
        return paths.BoardConfig(self.cfg.path, self.cfg.name, self.cfg.paper, data)

    def fails(self, edit, msg: str) -> None:
        errors = boardcheck.validate_config(self.with_data(edit), self.nl)
        self.assertTrue(any(msg in e for e in errors), errors)

    def test_balancebot_board_toml_is_valid(self):
        self.assertEqual(boardcheck.validate_config(self.cfg, self.nl), [])

    def test_class_expansion_order(self):
        classes = dict(boardcheck.expand_classes(self.cfg, self.nl))
        motors = [f"MOT{s}_{p}" for s in "LR" for p in ("A1", "A2", "B1", "B2")]
        self.assertEqual(classes["power"], ["VBAT", "GND", *motors])
        self.assertEqual(classes["logic_power"], ["+3V3", "BUCK_5V", "XIAO_5V"])

    def test_placement_completeness(self):
        self.fails(
            lambda d: d["placement"].pop("R2"), "[placement] has no entry for R2"
        )
        self.fails(
            lambda d: d["placement"].update(R9=[0, 0, 0]),
            "[placement] R9 is not in the netlist",
        )
        self.fails(
            lambda d: d["placement"].update(R2=[0, 0]),
            "R2 must be [x_mm, y_mm, rotation_deg]",
        )

    def test_net_classes(self):
        self.fails(
            lambda d: d["net_classes"][0]["nets"].append("NOPE*"),
            "'NOPE*' matches no net",
        )
        self.fails(
            lambda d: d["net_classes"][1]["nets"].append("GND"),
            "net 'GND' is in classes",
        )
        self.fails(
            lambda d: d["net_classes"][0].update(via_um=[1000]), "via_um must be"
        )

    def test_pour_and_silk(self):
        self.fails(
            lambda d: d["pour"].update(net="AGND"),
            "[pour] net 'AGND' is not in the netlist",
        )
        self.fails(
            lambda d: d["silk"]["ref_positions"].update(J9=[0, 0]),
            "ref_positions: 'J9'",
        )
        self.fails(lambda d: d["silk"]["ref_sizes"].update(U9=1.0), "ref_sizes: 'U9'")
        self.fails(lambda d: d["silk"]["hidden_refs"].append("H9"), "hidden_refs: 'H9'")
        self.fails(lambda d: d["silk"]["texts"].append({"text": "x"}), "[silk] texts")

    def test_unknown_and_missing_keys(self):
        self.fails(
            lambda d: d["pour"].update(clearence_mm=1),
            "[pour]: unknown key(s) ['clearence_mm']",
        )
        self.fails(lambda d: d.pop("signal"), "[signal] is required")

    def test_file_is_toml(self):
        self.assertEqual(
            tomllib.loads(self.cfg.path.read_text(encoding="utf-8")), self.cfg.data
        )

    def test_relink_rejects_a_footprint_without_one_path(self):
        text = board().replace('\t\t(path "', '\t\t(pathx "', 1)
        with self.assertRaisesRegex(KicadError, "0 \\(path"):
            boardcheck.relink(text, NS)


class MainTests(unittest.TestCase):
    """The CLI names the just module only when it prints a hint (#776)."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="kicad-boardcheck-")
        cls.root = Path(cls.tmp.name)
        cls.project = testfixture.materialise(cls.root)
        vendored = cls.root / library.VENDORED.relative_to(paths.REPO_ROOT)
        nl = netlist.build(cls.project, cls.root, vendored)
        ns = schematic.namespace(schematic.project_key(cls.project, cls.root))
        net = {node: name for name, nodes in nl.nets.items() for node in nodes}
        net |= {node: name for name, node in netlist.unconnected(nl, vendored).items()}
        fps = []
        for c in nl.comps:
            pads = [
                (num, name) for (ref, num), name in sorted(net.items()) if ref == c.ref
            ]
            fps.append(
                footprint(
                    c.ref,
                    c.footprint,
                    c.value,
                    c.lcsc,
                    pads,
                    boardcheck.symbol_path(ns, c.ref),
                )
            )
        cls.good = "(kicad_pcb\n\t(version 20260206)\n" + "\n".join(fps) + "\n)\n"
        cls.pcb = boardcheck.board_path(cls.project)
        cls.pcb.parent.mkdir(parents=True, exist_ok=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_main(self, justfile: str | None, text: str) -> tuple[int, str]:
        jf = self.root / "justfile"
        if justfile is None:
            jf.unlink(missing_ok=True)
        else:
            jf.write_text(justfile)
        self.pcb.write_text(text, encoding="utf-8")
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = boardcheck.main([str(self.project)], repo_root=self.root)
        return code, out.getvalue()

    def test_passing_board_with_the_module_named_unlike_the_directory(self):
        code, out = self.run_main("mod bb 'packages/robotics/balancebot'\n", self.good)
        self.assertEqual(code, 0, out)
        self.assertIn("OK: balancebot-carrier.kicad_pcb", out)

    def test_failing_board_hint_names_the_module_not_the_directory(self):
        bad = self.good.replace('(property "Value" "', '(property "Value" "x', 1)
        code, out = self.run_main("mod bb 'packages/robotics/balancebot'\n", bad)
        self.assertEqual(code, 1, out)
        self.assertIn("just bb::carrier-board", out)
        self.assertNotIn("balancebot::", out)

    def test_passing_board_needs_no_mod_line(self):
        for justfile in ("mod other 'packages/other'\n", None):
            with self.subTest(justfile=justfile):
                code, out = self.run_main(justfile, self.good)
                self.assertEqual(code, 0, out)
                self.assertIn("OK: balancebot-carrier.kicad_pcb", out)


if __name__ == "__main__":
    unittest.main()
