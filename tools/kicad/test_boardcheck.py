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

    def test_placement_side(self):
        def back(d):
            d["placement"]["R2"] = [*d["placement"]["R2"], "back"]

        self.assertEqual(boardcheck.validate_config(self.with_data(back), self.nl), [])
        self.fails(
            lambda d: d["placement"].update(R2=[0, 0, 0, "bottom"]),
            'R2 must be [x_mm, y_mm, rotation_deg] or [x_mm, y_mm, rotation_deg, "back"]',
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

    def test_keepouts(self):
        band = {"name": "cut band", "rect": [40.0, 0.0, 43.0, 62.0]}
        ok = self.with_data(lambda d: d.update(keepouts=[band]))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d.update(keepouts=[{**band, "rect": [40, 0, 43, 63]}]),
            "[keepouts] 'cut band': rect [40, 0, 43, 63] is not inside [outline]",
        )
        self.fails(
            lambda d: d.update(keepouts=[{**band, "rect": [43, 0, 40, 62]}]),
            "[keepouts] 'cut band': rect must be [x0, y0, x1, y1]",
        )
        self.fails(
            lambda d: d.update(keepouts=[{**band, "layer": "F.Cu"}]),
            "[keepouts]: unknown key(s) ['layer']",
        )
        self.fails(
            lambda d: d.update(keepouts=[{"rect": band["rect"]}]),
            "[keepouts] needs a name",
        )
        self.fails(
            lambda d: d.update(keepouts=[band, band]),
            "[keepouts] names repeat",
        )

    def test_cutouts(self):
        hole = {"name": "relief", "center": [41.5, 30.0], "diameter": 2.0}
        ok = self.with_data(lambda d: d.update(cutouts=[hole]))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d.update(cutouts=[{**hole, "center": [0.9, 30.0]}]),
            "[cutouts] 'relief': circle at [0.9, 30.0], diameter 2.0, is not inside",
        )
        self.fails(
            lambda d: d.update(cutouts=[{**hole, "center": [41.5, 61.0]}]),
            "[cutouts] 'relief': circle at [41.5, 61.0], diameter 2.0, is not inside",
        )
        self.fails(
            lambda d: d.update(cutouts=[{**hole, "diameter": 0}]),
            "[cutouts] 'relief': needs center = [x, y] and diameter > 0",
        )
        self.fails(
            lambda d: d.update(cutouts=[{**hole, "center": [41.5]}]),
            "[cutouts] 'relief': needs center = [x, y] and diameter > 0",
        )
        self.fails(
            lambda d: d.update(cutouts=[{**hole, "rect": [0, 0, 1, 1]}]),
            "[cutouts]: unknown key(s) ['rect']",
        )
        self.fails(
            lambda d: d.update(cutouts=[{"center": [41.5, 30.0], "diameter": 2.0}]),
            "[cutouts] needs a name",
        )
        self.fails(
            lambda d: d.update(cutouts=[hole, hole]),
            "[cutouts] names repeat",
        )

    def test_cut_lines(self):
        line = [41.5, 0.5, 41.5, 61.5]

        def cut(d):
            # a carrier's pour stops short of its cut lines (see below)
            del d["pour"]["net"]
            d["pour"]["regions"] = [
                {"net": "GND", "rect": [0, 0, 40.5, 62]},
                {"net": "GND", "rect": [42.5, 0, 90, 62]},
            ]
            d["silk"].update(cut_lines=[line])

        ok = self.with_data(cut)
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d["silk"].update(cut_lines=[[41.5, 0.5, 41.5]]),
            "[silk] cut_lines: [41.5, 0.5, 41.5] must be [x0, y0, x1, y1]",
        )
        self.fails(
            lambda d: d["silk"].update(cut_lines=[[41.5, 1, 41.5, 1]]),
            "[silk] cut_lines: [41.5, 1, 41.5, 1] has no length",
        )
        self.fails(
            lambda d: d["silk"].update(cut_lines=[[41.5, 0.5, 41.5, 70]]),
            "[silk] cut_lines: [41.5, 0.5, 41.5, 70] is not inside [outline]",
        )

    def test_pour_regions(self):
        def regions(d, rs):
            del d["pour"]["net"]
            d["pour"]["regions"] = rs

        gnd = {"net": "GND", "rect": [0, 0, 40, 62]}
        other = {"net": "+3V3", "rect": [43, 0, 90, 62]}
        ok = self.with_data(lambda d: regions(d, [gnd, other]))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d["pour"].update(regions=[gnd]),
            "[pour] takes net or regions, not both",
        )
        self.fails(lambda d: d["pour"].pop("net"), "[pour] needs net or regions")
        self.fails(
            lambda d: regions(d, [{**gnd, "net": "AGND"}]),
            "[pour] regions: net 'AGND' is not in the netlist",
        )
        self.fails(
            lambda d: regions(d, [gnd, {**other, "rect": [39, 0, 90, 62]}]),
            "[pour] regions: [0, 0, 40, 62] and [39, 0, 90, 62] overlap",
        )
        self.fails(
            lambda d: regions(d, [{**gnd, "rect": [0, 0, 40, 70]}]),
            "[pour] regions: rect [0, 0, 40, 70] is not inside [outline]",
        )
        self.fails(
            lambda d: regions(d, [{**gnd, "layer": "F.Cu"}]),
            "[pour] regions: unknown key(s) ['layer']",
        )
        self.fails(lambda d: regions(d, []), "[pour] regions must be a non-empty list")

    def test_pour_regions_clear_the_cut_lines(self):
        """A pour stops 1 mm short of every hand-saw cut line, so the saw (and
        the flex of sawing) cannot bridge its copper to the next piece's."""

        def setup(d, rect, line):
            del d["pour"]["net"]
            d["pour"]["regions"] = [{"net": "GND", "rect": rect}]
            d.setdefault("silk", {})["cut_lines"] = [line]

        ok = self.with_data(lambda d: setup(d, [0, 0, 40, 62], [41, 0.5, 41, 61]))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: setup(d, [0, 0, 40, 62], [40.5, 0.5, 40.5, 61]),
            "[pour] regions: [0, 0, 40, 62] is 0.5 mm from cut line "
            "[40.5, 0.5, 40.5, 61]; keep 1.0 mm",
        )
        # a whole-board [pour] (net, no regions) meets every cut line
        self.fails(
            lambda d: d.setdefault("silk", {}).update(cut_lines=[[41, 0.5, 41, 61]]),
            "[pour] regions: None is 0.0 mm from cut line [41, 0.5, 41, 61]",
        )

    def test_silk_text_height_floor(self):
        """JLCPCB prints 1.0 mm text legibly; 0.8 mm is the floor where a label
        will not fit at 1.0, and nothing goes below it."""
        self.fails(
            lambda d: d.setdefault("silk", {}).update(
                ref_sizes={next(iter(d["placement"])): 0.6}
            ),
            "is 0.6 mm tall; the floor is 0.8 mm",
        )
        self.fails(
            lambda d: d.setdefault("silk", {}).update(
                texts=[{"text": "X", "at": [5, 5], "size": 0.7}]
            ),
            "[silk] texts: 'X' is 0.7 mm tall; the floor is 0.8 mm",
        )

    def test_pour_region_layers_and_priority(self):
        """A region may name its own layers, and overlap one on a shared layer
        when both carry different priorities (a power pour over GND)."""

        def regions(d, rs):
            del d["pour"]["net"]
            d["pour"]["regions"] = rs

        gnd = {"net": "GND", "rect": [0, 0, 90, 62]}
        top = {"net": "+3V3", "rect": [10, 10, 40, 40], "layers": ["F.Cu"]}
        ok = self.with_data(
            lambda d: regions(d, [{**gnd, "priority": 0}, {**top, "priority": 1}])
        )
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        apart = self.with_data(
            lambda d: regions(
                d, [{**gnd, "layers": ["B.Cu"]}, {**top, "layers": ["F.Cu"]}]
            )
        )
        self.assertEqual(boardcheck.validate_config(apart, self.nl), [])
        self.fails(
            lambda d: regions(d, [gnd, top]),
            "[pour] regions: [0, 0, 90, 62] and [10, 10, 40, 40] overlap on F.Cu",
        )
        self.fails(
            lambda d: regions(d, [{**gnd, "priority": 1}, {**top, "priority": 1}]),
            "[pour] regions: [0, 0, 90, 62] and [10, 10, 40, 40] overlap on F.Cu",
        )
        self.fails(
            lambda d: regions(d, [{**top, "layers": ["In1.Cu"]}]),
            "[pour] regions: layers ['In1.Cu'] are not in [pour] layers",
        )
        self.fails(
            lambda d: regions(d, [{**top, "layers": []}]),
            "[pour] regions: layers must be a non-empty list",
        )
        self.fails(
            lambda d: regions(d, [{**top, "priority": -1}]),
            "[pour] regions: priority must be an integer >= 0, got -1",
        )

    def test_fanout(self):
        """[fanout]: a via beside every SMD pad of a poured net, after routing."""
        ref, pin = self.nl.nets["GND"][0]
        fan = {
            "nets": ["GND"],
            "via_um": [600, 300],
            "track_um": 300,
            "reach_mm": 2.0,
            "in_pad": [f"{ref}:{pin}"],
            "exempt": [],
            "required": [f"{ref}:{pin}"],
        }
        ok = self.with_data(lambda d: d.update(fanout=fan))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d.update(fanout={**fan, "nets": ["AGND"]}),
            "[fanout] net 'AGND' is not in the netlist",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "nets": []}),
            "[fanout] nets must be a non-empty list",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "via_um": [300, 300]}),
            "[fanout] via_um must be [diameter, drill] with diameter > drill > 0",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "track_um": 0}),
            "[fanout] track_um must be > 0",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "reach_mm": -1}),
            "[fanout] reach_mm must be > 0",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "in_pad": ["Q99:1"]}),
            "[fanout] in_pad 'Q99:1' is not a pad of a [fanout] net",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "exempt": ["nonsense"]}),
            "[fanout] exempt 'nonsense' is not a pad of a [fanout] net",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "required": ["Q99:2"]}),
            "[fanout] required 'Q99:2' is not a pad of a [fanout] net",
        )
        self.fails(
            lambda d: d.update(fanout={**fan, "stitch": True}),
            "[fanout]: unknown key(s) ['stitch']",
        )

    def test_tracks(self):
        """[[tracks]]: pre-routed tracks (a power trunk) Freerouting keeps."""
        trunk = {
            "name": "trunk",
            "net": "GND",
            "layer": "F.Cu",
            "width_mm": 1.5,
            "points": [[5, 5], [5, 30], [30, 30]],
        }
        ok = self.with_data(lambda d: d.update(tracks=[trunk]))
        self.assertEqual(boardcheck.validate_config(ok, self.nl), [])
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "net": "AGND"}]),
            "[[tracks]] 'trunk': net 'AGND' is not in the netlist",
        )
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "layer": "In1.Cu"}]),
            "[[tracks]] 'trunk': layer must be F.Cu or B.Cu, got 'In1.Cu'",
        )
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "width_mm": 0}]),
            "[[tracks]] 'trunk': width_mm must be > 0",
        )
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "points": [[5, 5]]}]),
            "[[tracks]] 'trunk': points must be two or more [x, y]",
        )
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "points": [[5, 5], [5, 99]]}]),
            "[[tracks]] 'trunk': point [5, 99] is not inside [outline]",
        )
        self.fails(
            lambda d: d.update(
                tracks=[trunk], keepouts=[{"name": "band", "rect": [20, 28, 22, 32]}]
            ),
            "[[tracks]] 'trunk': [5, 30] to [30, 30] enters keepout 'band'",
        )
        self.fails(
            lambda d: d.update(tracks=[{**trunk, "colour": "red"}]),
            "[tracks]: unknown key(s) ['colour']",
        )

    def test_pour_regions_of_the_net_and_whole_board(self):
        self.assertEqual(
            boardcheck.pour_regions(self.cfg.data["pour"]), [("GND", None)]
        )
        self.assertEqual(
            boardcheck.pour_regions(
                {"regions": [{"net": "GND", "rect": [0, 0, 1, 1]}]}
            ),
            [("GND", [0, 0, 1, 1])],
        )

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


ORIGIN = (100.0, 100.0)
BAND = {"name": "cut band", "rect": [40.0, 0.0, 43.0, 62.0]}  # x 140..143 absolute


def segment(x1, y1, x2, y2, width=0.3, layer="F.Cu", net="A") -> str:
    return (
        f"\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n"
        f'\t\t(width {width})\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t)'
    )


def via(x, y, size=0.6, net="A") -> str:
    return (
        f"\t(via\n\t\t(at {x} {y})\n\t\t(size {size})\n\t\t(drill 0.3)\n"
        f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t)'
    )


def arc(start, mid, end, width=0.3, net="A") -> str:
    return (
        f"\t(arc\n\t\t(start {start[0]} {start[1]})\n\t\t(mid {mid[0]} {mid[1]})\n"
        f"\t\t(end {end[0]} {end[1]})\n\t\t(width {width})\n"
        f'\t\t(layer "B.Cu")\n\t\t(net "{net}")\n\t)'
    )


def zone(points, layer="F.Cu", net="GND") -> str:
    pts = " ".join(f"(xy {x} {y})" for x, y in points)
    return (
        f'\t(zone\n\t\t(net "{net}")\n\t\t(layer "{layer}")\n'
        f"\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 100 100) (xy 190 100) (xy 190 162) (xy 100 162)\n\t\t\t)\n\t\t)\n"
        f'\t\t(filled_polygon\n\t\t\t(layer "{layer}")\n\t\t\t(pts\n\t\t\t\t{pts}\n\t\t\t)\n\t\t)\n\t)'
    )


def keepout_errors(*items: str) -> list[str]:
    text = "(kicad_pcb\n\t(version 20260206)\n" + "\n".join(items) + "\n)\n"
    return boardcheck.keepout_copper(text, [BAND], ORIGIN)


class KeepoutCopperTests(unittest.TestCase):
    """Tracks, vias and pour fill against board.toml [[keepouts]] (#750)."""

    def test_clear_board(self):
        self.assertEqual(
            keepout_errors(
                segment(110, 120, 139.8, 120),  # edge at 139.95, band from 140
                segment(143.2, 110, 160, 110),
                via(139.6, 130),  # radius 0.3: reaches 139.9
                arc((150, 120), (152, 122), (150, 124)),
                # Pour stopping exactly on the band's edge touches, not enters.
                zone([(100, 100), (140, 100), (140, 162), (100, 162)]),
            ),
            [],
        )

    def test_segment_crossing_the_band(self):
        errors = keepout_errors(segment(130, 120, 150, 120, net="SDA"))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("segment on 'SDA'", errors[0])
        self.assertIn("'cut band'", errors[0])
        self.assertIn("(30, 20)-(50, 20)", errors[0])  # relative to the origin

    def test_segment_whose_width_reaches_in(self):
        self.assertEqual(len(keepout_errors(segment(110, 120, 139.9, 120))), 1)

    def test_segment_inside_without_an_end_inside(self):
        # Runs along the band, both ends beyond it: no end point is inside.
        self.assertEqual(len(keepout_errors(segment(141.5, 90, 141.5, 170))), 1)

    def test_via_in_the_band(self):
        errors = keepout_errors(via(141, 130))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("via on 'A'", errors[0])

    def test_arc_bulging_into_the_band(self):
        errors = keepout_errors(arc((138, 120), (141, 125), (138, 130)))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("arc", errors[0])

    def test_pour_into_the_band(self):
        errors = keepout_errors(zone([(100, 100), (141, 100), (141, 162), (100, 162)]))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("pour on 'GND' (F.Cu)", errors[0])

    def test_pour_around_the_band_with_no_vertex_inside(self):
        errors = keepout_errors(zone([(100, 90), (190, 90), (190, 170), (100, 170)]))
        self.assertEqual(len(errors), 1, errors)

    def test_non_copper_zone_layer_is_ignored(self):
        self.assertEqual(
            keepout_errors(zone([(100, 90), (190, 90), (190, 170)], layer="F.Mask")),
            [],
        )


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

    def test_copper_in_a_keepout_fails(self):
        toml = paths.carrier_dir(self.project) / paths.BOARD_TOML
        kept = toml.read_text(encoding="utf-8")
        self.addCleanup(toml.write_text, kept, encoding="utf-8")
        toml.write_text(
            kept + '\n[[keepouts]]\nname = "band"\nrect = [40.0, 0.0, 43.0, 62.0]\n',
            encoding="utf-8",
        )
        justfile = "mod bb 'packages/robotics/balancebot'\n"
        code, out = self.run_main(justfile, self.good)
        self.assertEqual(code, 0, out)
        bad = self.good.replace(
            "\n)\n", "\n" + segment(130, 120, 150, 120, net="GND") + "\n)\n"
        )
        code, out = self.run_main(justfile, bad)
        self.assertEqual(code, 1, out)
        self.assertIn("keepout 'band'", out)
        self.assertIn("just bb::carrier-board", out)

    def test_passing_board_needs_no_mod_line(self):
        for justfile in ("mod other 'packages/other'\n", None):
            with self.subTest(justfile=justfile):
                code, out = self.run_main(justfile, self.good)
                self.assertEqual(code, 0, out)
                self.assertIn("OK: balancebot-carrier.kicad_pcb", out)


if __name__ == "__main__":
    unittest.main()
